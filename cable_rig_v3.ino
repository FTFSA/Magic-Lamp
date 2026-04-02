/*
 * Cable Rig Controller v2.2 - Dual TMC2209 UART Firmware
 * Arduino Nano RP2040 Connect
 * 
 * FIXES in v2.2:
 *   - Non-blocking wind/unwind (serial stays responsive)
 *   - Proper DIR pin setup time before STEP pulse
 *   - Reliable direction for pay out / unwind
 * 
 * PINOUT:
 *   Motor A (Left - vertical):  D2=STEP, D3=DIR, D4=EN
 *   Motor B (Right - horizontal): D5=STEP, D6=DIR, D7=EN
 *   Both share Serial1 TX (via 1kΩ each) for TMC2209 UART
 *   Driver A addr=0 (MS1=GND, MS2=GND)
 *   Driver B addr=1 (MS1=3.3V, MS2=GND)
 *   12V PSU → VM, Nano 3.3V → VIO, shared GND
 * 
 * REQUIRES: TMCStepper library (Arduino Library Manager)
 * 
 * PROTOCOL (115200 baud, newline-terminated):
 *   JL+100 / JL-100     Jog left ± steps
 *   JL+100 C            Jog left + coupled R
 *   JR+100 / JR-100     Jog right ± steps
 *   ML1000 / MR500      Absolute move
 *   MOVE L1000 R500 S800  Coordinated move
 *   SPEED 600           Steps/sec
 *   HOME                Set 0,0
 *   GOHOME              Return to 0,0
 *   POS                 Report positions
 *   STOP                Emergency stop
 *   ENABLE / DISABLE    Motor drivers
 *   WIND L/R            Continuous wind (send STOP to stop)
 *   UNWIND L/R          Continuous unwind (send STOP to stop)
 *   INVL 1/0            Invert left direction
 *   INVR 1/0            Invert right direction
 *   COUPLE 0.5          Coupling ratio (0=off)
 *   CURRENT 400         RMS current mA
 *   MICRO 16            Microstepping
 *   STEALTHCHOP 1/0     Quiet mode
 *   PING / STATUS
 */

#include <TMCStepper.h>

// ============ PINS ============
#define L_STEP  2
#define L_DIR   3
#define L_EN    4

#define R_STEP  5
#define R_DIR   6
#define R_EN    7

// ============ UART ============
#define SERIAL_PORT Serial1
#define DRIVER_BAUD 115200
#define R_SENSE     0.11f

TMC2209Stepper driverA(&SERIAL_PORT, R_SENSE, 0);
TMC2209Stepper driverB(&SERIAL_PORT, R_SENSE, 1);

// ============ STATE ============
volatile long posL = 0, posR = 0;
long targetL = 0, targetR = 0;

float stepsPerSec = 600.0;
uint16_t rmsCurrent = 300;
uint16_t microsteps = 16;
bool stealthChop = true;

bool invertL = false, invertR = false;
float coupleRatio = 0.0;

// Winding state
bool windingL = false, windingR = false;
int windDirL = 0, windDirR = 0;

// Coordinated move
bool coordMoving = false;
long coordTargetL = 0, coordTargetR = 0;
float coordSpeed = 0;

// Step timing (non-blocking)
unsigned long lastStepTimeL = 0, lastStepTimeR = 0;

bool motorsEnabled = true;
String inputBuffer = "";

// ============ SETUP ============

void setup() {
  Serial.begin(115200);
  
  pinMode(L_STEP, OUTPUT); pinMode(L_DIR, OUTPUT); pinMode(L_EN, OUTPUT);
  pinMode(R_STEP, OUTPUT); pinMode(R_DIR, OUTPUT); pinMode(R_EN, OUTPUT);
  
  // Set DIR pins to known state
  digitalWrite(L_DIR, LOW);
  digitalWrite(R_DIR, LOW);
  
  // Enable drivers
  digitalWrite(L_EN, LOW);
  digitalWrite(R_EN, LOW);
  
  // Init TMC UART
  SERIAL_PORT.begin(DRIVER_BAUD);
  delay(200);  // Let drivers fully boot
  
  configureDriver(driverA, 'A');
  configureDriver(driverB, 'B');
  
  delay(50);
  Serial.println("READY v2.2");
  reportStatus();
}

void configureDriver(TMC2209Stepper &drv, char label) {
  drv.begin();
  drv.toff(4);
  drv.blank_time(24);
  drv.rms_current(rmsCurrent);
  drv.microsteps(microsteps);
  drv.TCOOLTHRS(0xFFFFF);
  drv.semin(5);
  drv.semax(2);
  drv.shaft(false);
  
  if (stealthChop) {
    drv.en_spreadCycle(false);
  } else {
    drv.en_spreadCycle(true);
  }
  
  drv.pwm_autoscale(true);
  drv.pwm_autograd(true);
  
  uint8_t result = drv.test_connection();
  Serial.print("TMC "); Serial.print(label);
  if (result == 0) Serial.println(" OK");
  else { Serial.print(" FAIL:"); Serial.println(result); }
}

// ============ MAIN LOOP ============

void loop() {
  // ALWAYS read serial first — never block this
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      if (inputBuffer.length() < 64) {  // Prevent buffer overflow
        inputBuffer += c;
      }
    }
  }
  
  unsigned long now = micros();
  unsigned long stepDelay = calcDelay(stepsPerSec);
  
  // Continuous winding — NON-BLOCKING using timing
  if (windingL && now - lastStepTimeL >= stepDelay) {
    stepMotor('L', windDirL);
    lastStepTimeL = now;
  }
  if (windingR && now - lastStepTimeR >= stepDelay) {
    stepMotor('R', windDirR);
    lastStepTimeR = now;
  }
  
  // Coordinated move
  if (coordMoving) {
    runCoordMove();
    return;
  }
  
  // Individual target moves (only when not winding)
  if (!windingL && !windingR) {
    runMoves();
  }
}

// ============ COMMANDS ============

void processCommand(String cmd) {
  cmd.trim();
  String upper = cmd;
  upper.toUpperCase();
  
  if (upper == "PING") { Serial.println("PONG"); return; }
  
  if (upper == "STOP") {
    windingL = false; windingR = false;
    coordMoving = false;
    targetL = posL; targetR = posR;
    Serial.println("OK STOP");
    reportPosition();
    return;
  }
  
  if (upper == "POS") { reportPosition(); return; }
  if (upper == "STATUS") { reportStatus(); return; }
  
  if (upper == "HOME") {
    posL = 0; posR = 0; targetL = 0; targetR = 0;
    Serial.println("OK HOME");
    reportPosition();
    return;
  }
  
  if (upper == "GOHOME") {
    windingL = false; windingR = false;  // Stop any winding
    coordTargetL = 0; coordTargetR = 0;
    coordSpeed = stepsPerSec;
    coordMoving = true;
    Serial.println("OK GOHOME");
    return;
  }
  
  if (upper == "ENABLE") {
    digitalWrite(L_EN, LOW); digitalWrite(R_EN, LOW);
    motorsEnabled = true;
    Serial.println("OK ENABLED");
    return;
  }
  
  if (upper == "DISABLE") {
    digitalWrite(L_EN, HIGH); digitalWrite(R_EN, HIGH);
    motorsEnabled = false;
    windingL = false; windingR = false; coordMoving = false;
    Serial.println("OK DISABLED");
    return;
  }
  
  // DIRECTION INVERT
  if (upper.startsWith("INVL ")) {
    invertL = (upper.charAt(5) == '1');
    Serial.print("OK INVL "); Serial.println(invertL ? "ON" : "OFF");
    return;
  }
  if (upper.startsWith("INVR ")) {
    invertR = (upper.charAt(5) == '1');
    Serial.print("OK INVR "); Serial.println(invertR ? "ON" : "OFF");
    return;
  }
  
  // COUPLING
  if (upper.startsWith("COUPLE ")) {
    coupleRatio = cmd.substring(7).toFloat();
    coupleRatio = constrain(coupleRatio, 0.0, 5.0);
    Serial.print("OK COUPLE "); Serial.println(coupleRatio, 3);
    return;
  }
  
  // CURRENT
  if (upper.startsWith("CURRENT ")) {
    rmsCurrent = constrain(cmd.substring(8).toInt(), 100, 1200);
    driverA.rms_current(rmsCurrent);
    driverB.rms_current(rmsCurrent);
    Serial.print("OK CURRENT "); Serial.print(rmsCurrent); Serial.println("mA");
    return;
  }
  
  // MICROSTEPPING
  if (upper.startsWith("MICRO ")) {
    microsteps = cmd.substring(6).toInt();
    driverA.microsteps(microsteps);
    driverB.microsteps(microsteps);
    Serial.print("OK MICRO "); Serial.println(microsteps);
    return;
  }
  
  // STEALTHCHOP
  if (upper.startsWith("STEALTHCHOP ")) {
    stealthChop = (upper.charAt(12) == '1');
    driverA.en_spreadCycle(!stealthChop);
    driverB.en_spreadCycle(!stealthChop);
    Serial.print("OK STEALTH "); Serial.println(stealthChop ? "ON" : "OFF");
    return;
  }
  
  // JOG: JL+100, JR-200, JL+100 C
  if (upper.startsWith("JL") || upper.startsWith("JR")) {
    // Stop any winding before jogging
    windingL = false; windingR = false;
    
    char motor = upper.charAt(1);
    bool coupled = upper.indexOf('C') > 2;
    
    String numPart = "";
    for (int i = 2; i < (int)upper.length(); i++) {
      char ch = upper.charAt(i);
      if (ch == '+' || ch == '-' || (ch >= '0' && ch <= '9')) numPart += ch;
      else break;
    }
    long steps = numPart.toInt();
    
    if (motor == 'L') {
      targetL = posL + steps;
      if (coupled && coupleRatio > 0 && steps != 0) {
        long rSteps = (long)(abs(steps) * coupleRatio);
        targetR = posR + (steps > 0 ? rSteps : -rSteps);
      }
    } else {
      targetR = posR + steps;
    }
    coordMoving = false;
    Serial.print("OK JOG "); Serial.print(motor); Serial.print(" "); Serial.print(steps);
    if (coupled && motor == 'L') {
      Serial.print(" +R:"); Serial.print(targetR - posR);
    }
    Serial.println();
    return;
  }
  
  // ABSOLUTE MOVE
  if (upper.startsWith("ML") || upper.startsWith("MR")) {
    windingL = false; windingR = false;
    char motor = upper.charAt(1);
    long pos = cmd.substring(2).toInt();
    if (motor == 'L') targetL = pos; else targetR = pos;
    coordMoving = false;
    Serial.print("OK MOVE "); Serial.print(motor);
    Serial.print(" TO "); Serial.println(pos);
    return;
  }
  
  // SET POSITION
  if (upper.startsWith("SETL ") || upper.startsWith("SETR ")) {
    char motor = upper.charAt(3);
    long val = cmd.substring(5).toInt();
    if (motor == 'L') { posL = val; targetL = val; }
    else { posR = val; targetR = val; }
    Serial.print("OK SET "); Serial.print(motor);
    Serial.print("="); Serial.println(val);
    reportPosition();
    return;
  }
  
  // COORDINATED MOVE
  if (upper.startsWith("MOVE ")) {
    windingL = false; windingR = false;
    coordTargetL = posL; coordTargetR = posR;
    coordSpeed = stepsPerSec;
    
    int idx = upper.indexOf('L');
    if (idx >= 0) coordTargetL = extractNum(upper, idx + 1);
    idx = upper.indexOf('R');
    if (idx >= 0) coordTargetR = extractNum(upper, idx + 1);
    idx = upper.indexOf('S', 5);
    if (idx >= 0) coordSpeed = extractNum(upper, idx + 1);
    
    coordMoving = true;
    Serial.print("OK COORD L:"); Serial.print(coordTargetL);
    Serial.print(" R:"); Serial.print(coordTargetR);
    Serial.print(" S:"); Serial.println(coordSpeed);
    return;
  }
  
  // SPEED
  if (upper.startsWith("SPEED ")) {
    stepsPerSec = constrain(cmd.substring(6).toFloat(), 1, 5000);
    Serial.print("OK SPEED "); Serial.println(stepsPerSec);
    return;
  }
  
  // WIND / UNWIND
  if (upper.startsWith("WIND ") || upper.startsWith("UNWIND ")) {
    bool isWind = upper.startsWith("WIND");
    char motor = upper.charAt(upper.length() - 1);
    int dir = isWind ? 1 : -1;
    
    // Clear ALL other motion
    coordMoving = false;
    targetL = posL; targetR = posR;
    windingL = false; windingR = false;
    
    if (motor == 'L') { windingL = true; windDirL = dir; }
    else if (motor == 'R') { windingR = true; windDirR = dir; }
    
    Serial.print("OK "); Serial.print(isWind ? "WIND " : "UNWIND ");
    Serial.println(motor);
    return;
  }
  
  Serial.print("ERR "); Serial.println(cmd);
}

long extractNum(String s, int start) {
  String num = "";
  for (int i = start; i < (int)s.length(); i++) {
    char c = s.charAt(i);
    if (c == '-' || c == '+' || (c >= '0' && c <= '9')) num += c;
    else if (num.length() > 0) break;
  }
  return num.toInt();
}

// ============ MOTION ============

void runCoordMove() {
  long dL = coordTargetL - posL;
  long dR = coordTargetR - posR;
  
  if (dL == 0 && dR == 0) {
    coordMoving = false;
    Serial.println("DONE");
    reportPosition();
    return;
  }
  
  long absL = abs(dL), absR = abs(dR);
  long maxD = max(absL, absR);
  
  float spdL = maxD > 0 ? coordSpeed * ((float)absL / maxD) : 0;
  float spdR = maxD > 0 ? coordSpeed * ((float)absR / maxD) : 0;
  
  unsigned long now = micros();
  
  if (dL != 0 && spdL > 0 && now - lastStepTimeL >= calcDelay(spdL)) {
    stepMotor('L', dL > 0 ? 1 : -1);
    lastStepTimeL = now;
  }
  if (dR != 0 && spdR > 0 && now - lastStepTimeR >= calcDelay(spdR)) {
    stepMotor('R', dR > 0 ? 1 : -1);
    lastStepTimeR = now;
  }
}

void runMoves() {
  long dL = targetL - posL;
  long dR = targetR - posR;
  unsigned long now = micros();
  unsigned long d = calcDelay(stepsPerSec);
  
  if (dL != 0 && now - lastStepTimeL >= d) {
    stepMotor('L', dL > 0 ? 1 : -1);
    lastStepTimeL = now;
  }
  if (dR != 0 && now - lastStepTimeR >= d) {
    stepMotor('R', dR > 0 ? 1 : -1);
    lastStepTimeR = now;
  }
  
  static bool pML = false, pMR = false;
  bool mL = (dL != 0), mR = (dR != 0);
  if (pML && !mL) { Serial.println("DONEL"); reportPosition(); }
  if (pMR && !mR) { Serial.println("DONER"); reportPosition(); }
  pML = mL; pMR = mR;
}

// Step a motor one step in the given logical direction
// dir: +1 = reel in, -1 = pay out
void stepMotor(char motor, int dir) {
  if (!motorsEnabled) return;
  
  int stepPin, dirPin;
  bool inv;
  
  if (motor == 'L') {
    stepPin = L_STEP; dirPin = L_DIR; inv = invertL;
  } else {
    stepPin = R_STEP; dirPin = R_DIR; inv = invertR;
  }
  
  // Apply inversion
  int actualDir = inv ? -dir : dir;
  
  // Set direction pin FIRST with setup time
  digitalWrite(dirPin, actualDir > 0 ? HIGH : LOW);
  delayMicroseconds(5);  // TMC2209 needs ≥20ns DIR setup, 5μs is safe margin
  
  // Pulse STEP
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(3);  // TMC2209 needs ≥100ns pulse, 3μs is safe
  digitalWrite(stepPin, LOW);
  
  // Track logical position
  if (motor == 'L') posL += dir;
  else posR += dir;
}

unsigned long calcDelay(float speed) {
  if (speed <= 0) return 1000000;
  return (unsigned long)(1000000.0 / speed);
}

void reportPosition() {
  Serial.print("POS L:"); Serial.print(posL);
  Serial.print(" R:"); Serial.println(posR);
}

void reportStatus() {
  reportPosition();
  Serial.print("SPEED "); Serial.println(stepsPerSec);
  Serial.print("CURRENT "); Serial.print(rmsCurrent); Serial.println("mA");
  Serial.print("MICRO "); Serial.println(microsteps);
  Serial.print("STEALTH "); Serial.println(stealthChop ? "ON" : "OFF");
  Serial.print("INVL "); Serial.println(invertL ? "1" : "0");
  Serial.print("INVR "); Serial.println(invertR ? "1" : "0");
  Serial.print("COUPLE "); Serial.println(coupleRatio, 3);
  Serial.print("TMC_A "); Serial.println(driverA.test_connection() == 0 ? "OK" : "FAIL");
  Serial.print("TMC_B "); Serial.println(driverB.test_connection() == 0 ? "OK" : "FAIL");
}
