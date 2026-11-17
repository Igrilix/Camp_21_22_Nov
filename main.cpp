// ESP32 2-wheel WiFi car: TB6612FNG + 6 servo headers + buzzer
// Pins taken from the schematic. STBY is tied to 3V3 on the board.
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <esp_system.h>

// ---------- WiFi ----------
// Mode 1: join your router (fill STA_SSID). Falls back to AP if it fails.
// Mode 2: AP "ESP32-CAR" / pass "12345678" -> open http://192.168.4.1
const char* STA_SSID = "";          // e.g. "MyWiFi"  (leave empty for AP only)
const char* STA_PASS = "";
const char* AP_SSID  = "ESP32-CAR";
const char* AP_PASS  = "12345678";

// ---------- Pins ----------
const int PWMA = 21, AIN1 = 18, AIN2 = 19;   // motor A (left)
const int PWMB = 4,  BIN1 = 17, BIN2 = 16;   // motor B (right)
const int BUZZER = 15;
const int SERVO_PINS[7] = {22, 27, 25, 12, 13, 14, 23};  // S1 on GP22 (homing servo), S2 GP27, S3 GP25 (limit switch, unused), S4 GP12, S5 GP13, S6 GP14

// Flip if a wheel spins the wrong way / A,B are swapped left-right
const bool INVERT_A = false, INVERT_B = false;
const bool SWAP_LR  = false;

// ---------- Motor PWM (LEDC) ----------
const int CH_A = 14, CH_B = 15, PWM_FREQ = 20000, PWM_RES = 8;

// ---------- Homing: servo lowers until limit switch, that point = zero ----------
// NOT in the schematic: wire a limit switch between GP25 (J3) and GND (uses INPUT_PULLUP,
// pressed = LOW). Change LIMIT_PIN / HOME_SERVO as needed.
const int  LIMIT_PIN   = 25;   // = header J3 (signal pin 1 + GND pin 3); S3 servo is NOT usable
const int  HOME_SERVO  = 0;      // index 0 = S1 (GP27)
// Direction of "down": false = physical angle decreases toward the switch (180 -> 0),
// true = angle increases toward the switch (0 -> 180). Flip if the servo homes the wrong way.
bool HOME_INVERT = false;           // editable on the web page (saved)
// S1 is a 360-degree CONTINUOUS-ROTATION servo: pulse = speed/direction, not angle.
// STOP_US = pulse that makes it stand still (tune 1450..1550 if it creeps).
int  STOP_US = 1500;
int  HOME_SPEED_US = 80;        // offset from STOP_US while homing (small = very slow)
int  MOVE_SPEED_US = 120;       // offset while moving to an angle
float DEG_PER_SEC = 60.0;       // CALIBRATE: measured deg/sec at MOVE_SPEED_US
const unsigned long HOME_TIMEOUT_MS = 40000;
const int  HOME_STEP_US = 2;     // microseconds per step (1 deg = ~10 us)
const int  HOME_STEP_MS = 50;    // ms per step -> 2us/50ms = ~4 deg/s (full sweep ~45 s)
const int  HOME_BACKOFF = 6;     // deg to lift off the switch after hitting it
// Your switch reads HIGH when pressed (wired NC), so pressed level = HIGH.
// If you rewire to NO (pressed = LOW), set this to LOW.
int  LIMIT_PRESSED_LEVEL = HIGH;
bool limitPressed() { return digitalRead(LIMIT_PIN) == LIMIT_PRESSED_LEVEL; }
int  zeroOffset = 0;             // physical angle of logical 0 for HOME_SERVO
bool homed = false;
int  s1Mode = 0;          // X button: 0 idle, 1 going UP (until IR), 2 going DOWN (until limit switch)
bool atTop = false;       // S1 is at the top (IR position)
unsigned long s1T0 = 0;
const unsigned long S1_TIMEOUT_MS = 30000;
const char* s1Why = "-";                    // why S1 last stopped (shown on the web page)
const unsigned long S1_UP_MAX_MS = 6000;   // going UP from home: stop after this long

// IR sensors (digital out, e.g. obstacle/line module). NOT in schematic: plug into
// J2 (GP26): signal / 3V3 / GND.
const int IR_PIN = 26;
bool IR_ACTIVE_LOW = true;   // most IR modules pull LOW when they detect something
int servoAngle[7] = {0, 90, 90, 90, 90, 90, 0};   // last commanded (logical) angle

Servo servos[7];

// Event log over USB serial (read with tools/log_serial.py): "[millis] message"
void logEv(const char* fmt, ...) {
  char b[160];
  va_list a; va_start(a, fmt); vsnprintf(b, sizeof(b), fmt, a); va_end(a);
  Serial.printf("[%lu] %s\r\n", millis(), b);
}
const char* rstName(int r) {
  switch (r) {
    case 1: return "power-on"; case 2: return "external"; case 3: return "software";
    case 4: return "panic/crash"; case 5: return "int-wdt"; case 6: return "task-wdt";
    case 7: return "wdt"; case 8: return "deep-sleep"; case 9: return "BROWNOUT"; case 10: return "sdio";
    default: return "unknown";
  }
}
unsigned xCount = 0;   // how many X-button requests the board has received
WebServer server(80);
unsigned long lastCmd = 0;
const unsigned long FAILSAFE_MS = 800;  // stop if no command (phone lost)

void driveMotor(int in1, int in2, int ch, int speed, bool inv) {
  if (inv) speed = -speed;
  speed = constrain(speed, -255, 255);
  digitalWrite(in1, speed > 0);
  digitalWrite(in2, speed < 0);
  ledcWrite(ch, abs(speed));
}

// left/right: -255..255
void drive(int left, int right) {
  if (SWAP_LR) { int t = left; left = right; right = t; }
  driveMotor(AIN1, AIN2, CH_A, left,  INVERT_A);
  driveMotor(BIN1, BIN2, CH_B, right, INVERT_B);
}

// logical angle -> physical (only the homed servo is shifted)
void moveHomeTo(int a);
void hitBeep();
void setServo(int n, int a) {
  a = constrain(a, 0, 180);
  if (n == HOME_SERVO) { moveHomeTo(a); return; }
  if (!servos[n].attached()) {
    servos[n].setPeriodHertz(50);
    servos[n].attach(SERVO_PINS[n], 500, 2400);
  }
  servoAngle[n] = a;
  servos[n].write(a);
}

// 3 reads in a row must agree (debounce)
bool limitStable() {
  for (int i = 0; i < 3; i++) { if (!limitPressed()) return false; delay(2); }
  return true;
}

// Continuous servo helpers. dirToward: +1 = toward switch, -1 = away.
int lastHomeDir = 0;
void homeRun(int dirToward, int speedUs) {
  int towardSign = HOME_INVERT ? 1 : -1;
  int dir = towardSign * dirToward;
  if (!servos[HOME_SERVO].attached()) {
    servos[HOME_SERVO].setPeriodHertz(50);
    servos[HOME_SERVO].attach(SERVO_PINS[HOME_SERVO], 500, 2400);
    servos[HOME_SERVO].writeMicroseconds(STOP_US);
    delay(20);
  } else if (lastHomeDir != 0 && lastHomeDir != dir) {
    // Pause briefly at stop before reversing direction to prevent reverse current spike
    servos[HOME_SERVO].writeMicroseconds(STOP_US);
    delay(60);
  }
  // Soft-start ramp (4 steps) to prevent stall-current brownout reset
  for (int step = 1; step <= 4; step++) {
    int s = (speedUs * step) / 4;
    servos[HOME_SERVO].writeMicroseconds(STOP_US + dir * s);
    if (step < 4) delay(25);
  }
  lastHomeDir = dir;
}
void homeStop() {
  servos[HOME_SERVO].writeMicroseconds(STOP_US);
  delay(40);
  servos[HOME_SERVO].detach();   // no pulses = no creeping/buzzing
  lastHomeDir = 0;
}

bool homeServo() {
  s1Mode = 0;
  homed = false;
  if (limitPressed()) {
    Serial.println("HOMING REFUSED: limit switch already pressed at start");
    return false;
  }
  unsigned long t0 = millis();
  homeRun(+1, HOME_SPEED_US);              // creep toward the switch
  while (!limitStable()) {
    server.handleClient();
    if (millis() - t0 > HOME_TIMEOUT_MS) {
      homeStop();
      Serial.println("HOMING FAILED: limit switch not triggered (timeout)");
      return false;
    }
    delay(2);
  }
  homeStop();                              // STOP immediately on the switch
  hitBeep();
  delay(200);
  homeRun(-1, HOME_SPEED_US);              // back off the switch a little
  delay(300);
  homeStop();
  zeroOffset = 0;
  servoAngle[HOME_SERVO] = 0;
  homed = true;
  Serial.println("Homed: stopped on limit switch, zero set");
  return true;
}

// Time-based move (a continuous servo has no position feedback)
void moveHomeTo(int a) {
  if (!homed) return;
  s1Mode = 0;
  a = constrain(a, 0, 180);
  int cur = servoAngle[HOME_SERVO];
  int d = a - cur;
  if (d == 0) return;
  unsigned long dur = (unsigned long)(abs(d) / DEG_PER_SEC * 1000.0);
  homeRun(d < 0 ? +1 : -1, MOVE_SPEED_US);
  bool hitSwitch = false;
  unsigned long t0 = millis();
  while (millis() - t0 < dur) {
    if (d < 0 && limitStable()) { a = 0; hitSwitch = true; break; }   // never push past the switch
    server.handleClient();
    delay(5);
  }
  homeStop();
  if (hitSwitch) hitBeep();
  servoAngle[HOME_SERVO] = a;
}

// ---------- Settings saved in flash (editable from the web page) ----------
Preferences prefs;
bool AUTO_HOME = true;
int O_OPEN = 90, O_CLOSE = 0;   // angles for the O-button servo (S7, GP23), set on the web page
bool oOn = false;   // home S1 automatically at boot
void loadCfg() {
  prefs.begin("car", false);
  HOME_INVERT  = prefs.getBool("invert", HOME_INVERT);
  AUTO_HOME    = prefs.getBool("autoHome", AUTO_HOME);
  STOP_US      = prefs.getInt("stopUs", STOP_US);
  HOME_SPEED_US= prefs.getInt("homeSpd", HOME_SPEED_US);
  MOVE_SPEED_US= prefs.getInt("moveSpd", MOVE_SPEED_US);
  DEG_PER_SEC  = prefs.getFloat("dps", DEG_PER_SEC);
  LIMIT_PRESSED_LEVEL = prefs.getBool("limHigh", LIMIT_PRESSED_LEVEL == HIGH) ? HIGH : LOW;
  IR_ACTIVE_LOW= prefs.getBool("irLow", IR_ACTIVE_LOW);
  O_OPEN  = prefs.getInt("oOpen", O_OPEN);
  O_CLOSE = prefs.getInt("oClose", O_CLOSE);
}
void saveCfg() {
  prefs.putBool("invert", HOME_INVERT);
  prefs.putBool("autoHome", AUTO_HOME);
  prefs.putInt("stopUs", STOP_US);
  prefs.putInt("homeSpd", HOME_SPEED_US);
  prefs.putInt("moveSpd", MOVE_SPEED_US);
  prefs.putFloat("dps", DEG_PER_SEC);
  prefs.putBool("limHigh", LIMIT_PRESSED_LEVEL == HIGH);
  prefs.putBool("irLow", IR_ACTIVE_LOW);
  prefs.putInt("oOpen", O_OPEN);
  prefs.putInt("oClose", O_CLOSE);
}

bool irHit() { return (digitalRead(IR_PIN) == LOW) == IR_ACTIVE_LOW; }

bool limPrev = false;
void hitBeep() { digitalWrite(BUZZER, HIGH); delay(150); digitalWrite(BUZZER, LOW); limPrev = true; }

void beep(int ms) {
  digitalWrite(BUZZER, HIGH); delay(ms); digitalWrite(BUZZER, LOW);
}

// IR noise meter: counts level changes per second (a clean sensor only flips when something passes)
bool irLast = false; unsigned irFlipCnt = 0, irFlipsLast = 0; unsigned long irWinT0 = 0;
void irSample() {
  bool v = irHit();
  if (v != irLast) { irFlipCnt++; irLast = v; logEv("IR -> %d", v); }
  if (millis() - irWinT0 >= 1000) { irFlipsLast = irFlipCnt; irFlipCnt = 0; irWinT0 = millis(); }
}

// ---------- Triangle / X buttons: S1 up (toggle with T) / down (X until IR toggles or home switch) ----------
bool s1IrStart = false;   // IR level when going DOWN started; we stop when it CHANGES
int  s1IrCnt = 0;
unsigned long s1IrT = 0;
const unsigned long IR_HOLD_MS = 150;   // new IR level must also hold this long
const int IR_CONFIRM = 8;    // consecutive reads with the new IR level before stopping
void s1Start(int mode) {
  s1Mode = mode; s1T0 = millis();
  logEv("S1 start %s (limit=%d ir=%d)", mode == 1 ? "UP" : "DOWN", limitPressed(), irHit());
  s1IrStart = irHit(); s1IrCnt = 0;
  homeRun(mode == 1 ? -1 : +1, MOVE_SPEED_US);   // UP = away from switch, DOWN = toward it
}
void s1Update() {                         // call from loop(), non-blocking
  if (!s1Mode) return;
  unsigned long el = millis() - s1T0;
  if (s1Mode == 2) {                      // Going DOWN: stop on limit switch OR IR change
    if (limitStable()) {
      s1Why = "home switch pressed"; logEv("S1 stop: %s after %lums", s1Why, el); homeStop(); s1Mode = 0; atTop = false; homed = true; servoAngle[HOME_SERVO] = 0; hitBeep(); return;
    }
    if (el > 400) {                       // IR must TOGGLE vs. its level at start (debounced)
      if (irHit() != s1IrStart) { if (!s1IrCnt) s1IrT = millis(); s1IrCnt++; } else s1IrCnt = 0;
      if (s1IrCnt >= IR_CONFIRM && millis() - s1IrT >= IR_HOLD_MS) { s1Why = "IR toggled"; logEv("S1 stop: %s after %lums", s1Why, el); homeStop(); s1Mode = 0; atTop = false; hitBeep(); return; }
    }
  }
  if (el > S1_TIMEOUT_MS) { s1Why = "30 s timeout"; logEv("S1 stop: %s after %lums", s1Why, el); homeStop(); s1Mode = 0; beep(600); }   // safety timeout
}

const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no,viewport-fit=cover">
<title>ESP32 CAR</title><style>
*{box-sizing:border-box}
body{margin:0;background:#111;color:#eee;font-family:sans-serif;touch-action:none;user-select:none;-webkit-user-select:none;overflow:hidden;height:100vh}
#pad{display:flex;align-items:center;justify-content:space-between;height:100vh;padding:0 4vw}
#joy{width:min(66vh,38vw);height:min(66vh,38vw);border-radius:50%;background:#222;border:3px solid #2a6;position:relative;touch-action:none;flex:none}
#knob{position:absolute;width:38%;height:38%;border-radius:50%;background:#2a6;pointer-events:none;left:31%;top:31%}
#mid{flex:1;text-align:center;padding:0 2vw;font-size:14px;line-height:1.5}
#face{display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(3,1fr);width:min(66vh,38vw);height:min(66vh,38vw);flex:none;gap:2%}
.fb{border-radius:50%;border:3px solid;background:#222;font-size:min(9vh,6vw);display:flex;align-items:center;justify-content:center;touch-action:none}
#bT{grid-area:1/2;color:#4c8;border-color:#4c8}#bS{grid-area:2/1;color:#e8a;border-color:#e8a}
#bO{grid-area:2/3;color:#e55;border-color:#e55}#bX{grid-area:3/2;color:#58f;border-color:#58f}
button.m{font-size:14px;padding:8px 12px;border:0;border-radius:8px;background:#444;color:#fff;margin:2px}
input[type=range]{width:90%}
#rot{display:none;position:fixed;inset:0;background:#111;z-index:9;align-items:center;justify-content:center;font-size:22px;text-align:center}
@media (orientation:portrait){#rot{display:flex}}
#menu{display:none;position:fixed;inset:0;background:#111;z-index:20;padding:10px;overflow:auto;touch-action:pan-y;text-align:center}
.r{max-width:360px;margin:6px auto;text-align:left}
</style></head><body>
<div id=rot>&#8635; หมุนจอเป็นแนวนอน<br>Rotate to landscape<br><small>(แตะเพื่อปิด)</small></div>
<div id=pad>
 <div id=joy><div id=knob></div></div>
 <div id=mid>
  <div id=st2>...</div>
  <canvas id=irc width=240 height=36 style="background:#000;border-radius:6px"></canvas>
  <div id=msg style="color:#fc3;min-height:20px"></div>
  Speed <span id=sv>200</span><br><input id=sp type=range min=50 max=255 value=200 oninput="sv.innerText=this.value">
  <div><button class=m id=fs>Fullscreen</button><button class=m id=gear>&#9881; Menu</button><button class=m id=stop style="background:#c33">STOP</button></div>
 </div>
 <div id=face>
  <div class=fb id=bT data-b=T>&#9651;</div>
  <div class=fb id=bS data-b=S>&#9633;</div>
  <div class=fb id=bO data-b=O>&#9675;</div>
  <div class=fb id=bX data-b=X>&#10005;</div>
 </div>
</div>
<div id=menu>
 <button class=m onclick="menu.style.display='none'" style="background:#2a6">&#10005; Close</button>
 <h3>Status</h3><div id=st style="font-size:16px">...</div>
 <h3>S1</h3>
 <button class=m onclick="homeMsg.innerText='homing...';fetch('/home').then(r=>r.text()).then(t=>homeMsg.innerText=t)">HOME S1</button> <span id=homeMsg></span><br>
 <button class=m id=s1u>&#9650; away</button><button class=m id=s1d>&#9660; to switch</button>
 <h3>Servos</h3><div id=sv6></div>
 <h3>Settings (saved in flash)</h3><div id=cfg></div><div id=cm></div>
</div>
<script>
const $=id=>document.getElementById(id);
const joy=$('joy'),knob=$('knob');let jx=0,jy=0,jt=null,act=false,dBusy=false,stBusy=false;
function jset(x,y){const R=joy.clientWidth/2,k=knob.offsetWidth/2;knob.style.left=(R-k+x*(R-k))+'px';knob.style.top=(R-k+y*(R-k))+'px'}
function jpos(e){const r=joy.getBoundingClientRect(),R=r.width/2;let x=(e.clientX-r.left-R)/R,y=(e.clientY-r.top-R)/R;const m=Math.hypot(x,y);if(m>1){x/=m;y/=m}jx=x;jy=-y;jset(x,y)}
function jsend(){if(dBusy)return;const s=+$('sp').value;const mag=Math.min(1,Math.hypot(jx,jy));if(mag<0.1){dBusy=true;fetch('/drive?l=0&r=0').catch(()=>{}).finally(()=>dBusy=false);return}
 let L=jy+jx,R=jy-jx;const k=mag/Math.max(Math.abs(L),Math.abs(R),1e-6);L=Math.round(L*k*s);R=Math.round(R*k*s);dBusy=true;fetch('/drive?l='+L+'&r='+R).catch(()=>{}).finally(()=>dBusy=false)}
function jend(){act=false;clearInterval(jt);jx=jy=0;jset(0,0);fetch('/drive?l=0&r=0').catch(()=>{})}
joy.addEventListener('pointerdown',e=>{e.preventDefault();joy.setPointerCapture(e.pointerId);act=true;jpos(e);jsend();jt=setInterval(jsend,120)});
joy.addEventListener('pointermove',e=>{if(act){e.preventDefault();jpos(e)}});
joy.addEventListener('pointerup',jend);joy.addEventListener('pointercancel',jend);
$('stop').onclick=()=>fetch('/drive?l=0&r=0');
document.querySelectorAll('.fb').forEach(b=>{
 b.addEventListener('pointerdown',e=>{e.preventDefault();b.style.background='#444';fetch('/btn?b='+b.dataset.b).then(r=>r.text()).then(t=>$('msg').innerText=t).catch(()=>{})});
 const up=()=>b.style.background='#222';b.addEventListener('pointerup',up);b.addEventListener('pointercancel',up);b.addEventListener('pointerleave',up)});
$('gear').onclick=()=>$('menu').style.display='block';
$('rot').onclick=()=>$('rot').style.display='none';
$('fs').onclick=()=>{const d=document.documentElement;(d.requestFullscreen?d.requestFullscreen():Promise.resolve()).then(()=>{if(screen.orientation&&screen.orientation.lock)return screen.orientation.lock('landscape')}).catch(()=>{})};
const irh=[];function drawIr(){const c=$('irc'),g=c.getContext('2d');g.fillStyle='#000';g.fillRect(0,0,c.width,c.height);g.strokeStyle='#5f5';g.beginPath();irh.forEach((v,i)=>{const x=i*c.width/60,y=v?5:c.height-5;i?g.lineTo(x,y):g.moveTo(x,y)});g.stroke()}
setInterval(()=>{if(stBusy)return;stBusy=true;fetch('/status').then(r=>r.json()).then(j=>{
 const m=['idle','going UP','going DOWN'][j.x];
 const h='Limit: '+(j.limit?'<b style=color:#f55>PRESSED</b>':'open')+' (GP25='+j.raw+') | IR: '+(j.ir?'<b style=color:#5f5>DETECT</b>':'clear')+' ('+j.irf+' flips/s'+(j.irf>4?' <b style=color:#f55>NOISY</b>':'')+')<br>S1: '+m+(j.top?' (TOP)':'')+' | homed: '+(j.homed?'yes':'NO')+'<br>Servo: '+j.s.join(', ')+'<br>S1 last stop: '+j.why+'<br>Uptime: '+j.up+'s | last reset: '+(({1:'power-on',2:'external',3:'software',4:'panic/crash',5:'int-wdt',6:'task-wdt',7:'wdt',8:'deep-sleep',9:'BROWNOUT',10:'sdio'})[j.rst]||j.rst);
 $('st2').innerHTML=h;$('st').innerHTML=h;irh.push(j.ir);if(irh.length>60)irh.shift();drawIr()}).catch(()=>{}).finally(()=>stBusy=false)},400);
function holdS1(id,d){const e=$(id);let t=null;const go=()=>fetch('/jog?d='+d+'&ms=200').catch(()=>{});
 e.addEventListener('pointerdown',ev=>{ev.preventDefault();go();t=setInterval(go,220)});
 const off=ev=>{ev.preventDefault();clearInterval(t);fetch('/jog?d=0').catch(()=>{})};
 e.addEventListener('pointerup',off);e.addEventListener('pointerleave',off);e.addEventListener('pointercancel',off)}
holdS1('s1u',-1);holdS1('s1d',1);
for(let i=0;i<7;i++){$('sv6').insertAdjacentHTML('beforeend',
 `<div class=r>S${i+1}: <span id=v${i}>90</span>&deg;<br><input type=range min=0 max=180 value=90 oninput="v${i}.innerText=this.value;fetch('/servo?n=${i}&a='+this.value)"></div>`)}
const F=[['homeSpd','Home speed (us)'],['moveSpd','Move speed (us)'],['stopUs','Stop pulse (us)'],['dps','Deg per sec']];
const C=[['autoHome','Auto-home at boot'],['invert','Home direction inverted'],['limHigh','Limit pressed = HIGH'],['irLow','IR active LOW']];
fetch('/cfg').then(r=>r.json()).then(j=>{let h='';
 F.forEach(f=>h+=`<div class=r>${f[1]} <input type=number step=any value=${j[f[0]]} style="width:90px" onchange="fetch('/set?k=${f[0]}&v='+this.value).then(r=>r.text()).then(t=>cm.innerText=t)"></div>`);
 C.forEach(c=>h+=`<div class=r><label><input type=checkbox ${j[c[0]]?'checked':''} onchange="fetch('/set?k=${c[0]}&v='+(this.checked?1:0)).then(r=>r.text()).then(t=>cm.innerText=t)"> ${c[1]}</label></div>`);
 h+=`<div class=r>Gripper (GP23) TARGET angle, press O: <span id=oo>${j.oOpen}</span>&deg;<br><input type=range min=0 max=180 value=${j.oOpen} oninput="oo.innerText=this.value;fetch('/servo?n=6&a='+this.value)" onchange="fetch('/set?k=oOpen&v='+this.value).then(r=>r.text()).then(t=>cm.innerText=t)"></div>`;
 h+=`<div class=r>Gripper (GP23) START angle, boot and O again: <span id=oc>${j.oClose}</span>&deg;<br><input type=range min=0 max=180 value=${j.oClose} oninput="oc.innerText=this.value;fetch('/servo?n=6&a='+this.value)" onchange="fetch('/set?k=oClose&v='+this.value).then(r=>r.text()).then(t=>cm.innerText=t)"></div>`;
 $('cfg').innerHTML=h});
</script></body></html>
)HTML";

void setup() {
  Serial.begin(115200);
  loadCfg();
  logEv("BOOT reset reason=%d (%s)", (int)esp_reset_reason(), rstName((int)esp_reset_reason()));
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(BUZZER, OUTPUT); digitalWrite(BUZZER, LOW);
  ledcSetup(CH_A, PWM_FREQ, PWM_RES); ledcAttachPin(PWMA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES); ledcAttachPin(PWMB, CH_B);
  drive(0, 0);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT);

  bool ok = false;
  if (strlen(STA_SSID)) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    WiFi.begin(STA_SSID, STA_PASS);
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print('.'); }
    ok = WiFi.status() == WL_CONNECTED;
    if (ok) Serial.printf("\nJoined %s -> http://%s\n", STA_SSID, WiFi.localIP().toString().c_str());
  }
  if (!ok) {
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("AP %s (pass %s) -> http://%s\n", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  }

  // Only attach the gripper servo (S7) after Wi-Fi is stable; other servos attach on demand in setServo()
  delay(100);
  setServo(6, O_CLOSE);

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/drive", []() {
    drive(server.arg("l").toInt(), server.arg("r").toInt());
    lastCmd = millis();
    server.send(200, "text/plain", "ok");
  });
  server.on("/servo", []() {
    int n = server.arg("n").toInt(), a = server.arg("a").toInt();
    if (n >= 0 && n < 7 && SERVO_PINS[n] != LIMIT_PIN && (n != HOME_SERVO || homed)) setServo(n, a);
    server.send(200, "text/plain", "ok");
  });
  server.on("/home", []() { logEv("HOME requested"); drive(0, 0); server.send(200, "text/plain", homeServo() ? "homed" : "failed (switch already pressed, or never hit)"); });
  server.on("/status", []() {
    char b[360];
    snprintf(b, sizeof(b),
      "{\"why\":\"%s\",\"up\":%lu,\"rst\":%d,\"limit\":%d,\"raw\":%d,\"homed\":%d,\"zero\":%d,\"ir\":%d,\"irf\":%u,\"x\":%d,\"top\":%d,\"s\":[%d,%d,%d,%d,%d,%d,%d]}",
      s1Why, millis() / 1000, (int)esp_reset_reason(), limitPressed(), digitalRead(LIMIT_PIN), homed, zeroOffset, irHit(), irFlipsLast, s1Mode, atTop,
      servoAngle[0], servoAngle[1], servoAngle[2], servoAngle[3], servoAngle[4], servoAngle[5], servoAngle[6]);
    server.send(200, "application/json", b);
  });
  server.on("/cfg", []() {
    char b[340];
    snprintf(b, sizeof(b),
      "{\"invert\":%d,\"autoHome\":%d,\"stopUs\":%d,\"homeSpd\":%d,\"moveSpd\":%d,\"dps\":%.1f,\"limHigh\":%d,\"irLow\":%d,\"oOpen\":%d,\"oClose\":%d}",
      HOME_INVERT, AUTO_HOME, STOP_US, HOME_SPEED_US, MOVE_SPEED_US, DEG_PER_SEC,
      LIMIT_PRESSED_LEVEL == HIGH, IR_ACTIVE_LOW, O_OPEN, O_CLOSE);
    server.send(200, "application/json", b);
  });
  server.on("/set", []() {
    String k = server.arg("k"); float v = server.arg("v").toFloat();
    if (k == "homeSpd") HOME_SPEED_US = constrain((int)v, 0, 400);
    else if (k == "moveSpd") MOVE_SPEED_US = constrain((int)v, 0, 400);
    else if (k == "stopUs") STOP_US = constrain((int)v, 1300, 1700);
    else if (k == "dps") DEG_PER_SEC = max(1.0f, v);
    else if (k == "invert") HOME_INVERT = v > 0;
    else if (k == "autoHome") AUTO_HOME = v > 0;
    else if (k == "limHigh") LIMIT_PRESSED_LEVEL = v > 0 ? HIGH : LOW;
    else if (k == "irLow") IR_ACTIVE_LOW = v > 0;
    else if (k == "oOpen") O_OPEN = constrain((int)v, 0, 180);
    else if (k == "oClose") O_CLOSE = constrain((int)v, 0, 180);
    else { server.send(400, "text/plain", "unknown key"); return; }
    saveCfg();
    server.send(200, "text/plain", "saved");
  });
  // Jog S1: d=1 toward switch, d=-1 away, d=0 stop. Tracks position if homed.
  server.on("/jog", []() {
    int d = server.arg("d").toInt();
    s1Mode = 0;
    int ms = constrain(server.arg("ms").toInt(), 0, 1500);
    if (d == 0) { homeStop(); server.send(200, "text/plain", "stopped"); return; }
    if (d > 0 && limitPressed()) { hitBeep(); server.send(200, "text/plain", "at limit"); return; }
    homeRun(d > 0 ? 1 : -1, MOVE_SPEED_US);
    unsigned long t0 = millis();
    bool hit = false;
    while (millis() - t0 < (unsigned long)ms) { if (d > 0 && limitStable()) { hit = true; break; } delay(5); }
    unsigned long el = millis() - t0;
    homeStop();
    if (homed) {
      float a = servoAngle[HOME_SERVO] + (d > 0 ? -1 : 1) * el / 1000.0 * DEG_PER_SEC;
      servoAngle[HOME_SERVO] = hit ? 0 : constrain((int)a, 0, 180);
    }
    if (hit) hitBeep();
    server.send(200, "text/plain", hit ? "HIT limit" : "ok");
  });
  // Controller buttons: T (triangle) = S1 up/stop toggle, X = S1 down until IR change or limit switch, O = gripper.
  server.on("/btn", []() {
    String b = server.arg("b");
    if (b == "T") {
      logEv("BTN T received mode=%d limit=%d ir=%d", s1Mode, limitPressed(), irHit());
      if (s1Mode == 1) {
        s1Why = "triangle stop";
        logEv("S1 stop: %s", s1Why);
        homeStop();
        s1Mode = 0;
        server.send(200, "text/plain", "T: S1 STOPPED");
      } else {
        s1Start(1);
        server.send(200, "text/plain", "T: S1 going UP (press T again to stop)");
      }
    } else if (b == "X") {
      logEv("BTN X received (#%u) mode=%d atTop=%d limit=%d ir=%d", ++xCount, s1Mode, atTop, limitPressed(), irHit());
      s1Start(2);
      logEv("  -> mode=%d", s1Mode);
      server.send(200, "text/plain", "X: S1 going DOWN (until IR or limit switch)");
    } else if (b == "O") {                    // O = toggle S7 servo open / close
      oOn = !oOn;
      int a = oOn ? O_OPEN : O_CLOSE;
      setServo(6, a);
      server.send(200, "text/plain", String("O: servo ") + (oOn ? "TARGET " : "START ") + a + " deg");
    } else {
      server.send(200, "text/plain", b + ": not assigned yet");
    }
  });
  server.on("/beep", []() { beep(150); server.send(200, "text/plain", "ok"); });
  server.begin();
  beep(80);
  if (AUTO_HOME) {                       // auto-calibrate S1 at boot (toggle on the web page)
    if (homeServo()) { beep(80); delay(80); beep(80); }   // 2 beeps = homed
    else beep(600);                                        // long beep = failed
  }
}

void loop() {
  server.handleClient();
  s1Update();
  irSample();
  bool nowLim = limitPressed();                 // buzzer when the limit switch is hit
  if (nowLim && !limPrev) hitBeep();
  limPrev = nowLim;
  static bool logLim = false;
  if (nowLim != logLim) { logLim = nowLim; logEv("LIMIT %s", nowLim ? "PRESSED" : "released"); }
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.printf("limit=%s homed=%d zero=%d ir(GP%d)=%d servo=%d,%d,%d,%d,%d,%d%s",
      limitPressed() ? "PRESSED" : "open", homed, zeroOffset,
      IR_PIN, irHit(),
      servoAngle[0], servoAngle[1], servoAngle[2], servoAngle[3], servoAngle[4], servoAngle[5], "\r\n");
  }
  if (millis() - lastCmd > FAILSAFE_MS) drive(0, 0);  // failsafe
}
