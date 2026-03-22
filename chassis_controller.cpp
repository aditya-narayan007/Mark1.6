/*
 * =========================================================
 *  Mark 1.6 — 4WD Chassis Controller
 *  Arduino Uno (dedicated to drivetrain)
 * =========================================================
 *  Receives UART commands from the Master board on Serial
 *  Controls 4 DC motors via two L298N drivers
 *
 *  Motor layout (top-down):
 *    FL ── FR
 *    |      |
 *    RL ── RR
 *
 *  Command Protocol (same as master):
 *    M F <speed>   Forward
 *    M B <speed>   Backward
 *    M L <speed>   Turn left (pivot)
 *    M R <speed>   Turn right (pivot)
 *    M S 0         Stop
 *    M X <speed>   Strafe left  (mecanum, if equipped)
 *    M Y <speed>   Strafe right (mecanum, if equipped)
 * =========================================================
 */

// ── Motor Driver A (L298N #1) — Left side ────────────────
const int FL_EN  = 3;   // Front-left PWM enable
const int FL_IN1 = 22;
const int FL_IN2 = 23;
const int RL_EN  = 5;   // Rear-left PWM enable
const int RL_IN1 = 26;
const int RL_IN2 = 27;

// ── Motor Driver B (L298N #2) — Right side ───────────────
const int FR_EN  = 9;   // Front-right PWM enable
const int FR_IN1 = 24;
const int FR_IN2 = 25;
const int RR_EN  = 11;  // Rear-right PWM enable
const int RR_IN1 = 28;
const int RR_IN2 = 29;

// ── Speed limits ──────────────────────────────────────────
const int MIN_PWM   = 60;
const int MAX_PWM   = 220;
const int TURN_PWM  = 150;  // Fixed PWM for pivot turns

// ── State ─────────────────────────────────────────────────
char cmdBuf[32];
uint8_t cmdLen = 0;
char currentDir = 'S';
int  currentSpd = 0;

// ── Motor primitives ──────────────────────────────────────
void motorSet(int en, int in1, int in2, int pwm, bool forward) {
  pwm = constrain(pwm, 0, MAX_PWM);
  digitalWrite(in1, forward  ? HIGH : LOW);
  digitalWrite(in2, forward  ? LOW  : HIGH);
  analogWrite(en, pwm);
}

void motorBrake(int en, int in1, int in2) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  analogWrite(en, 0);
}

// ── Drive commands ────────────────────────────────────────
void driveForward(int spd) {
  motorSet(FL_EN, FL_IN1, FL_IN2, spd, true);
  motorSet(RL_EN, RL_IN1, RL_IN2, spd, true);
  motorSet(FR_EN, FR_IN1, FR_IN2, spd, true);
  motorSet(RR_EN, RR_IN1, RR_IN2, spd, true);
}

void driveBackward(int spd) {
  motorSet(FL_EN, FL_IN1, FL_IN2, spd, false);
  motorSet(RL_EN, RL_IN1, RL_IN2, spd, false);
  motorSet(FR_EN, FR_IN1, FR_IN2, spd, false);
  motorSet(RR_EN, RR_IN1, RR_IN2, spd, false);
}

void pivotLeft(int spd) {
  // Left wheels backward, right wheels forward
  motorSet(FL_EN, FL_IN1, FL_IN2, spd, false);
  motorSet(RL_EN, RL_IN1, RL_IN2, spd, false);
  motorSet(FR_EN, FR_IN1, FR_IN2, spd, true);
  motorSet(RR_EN, RR_IN1, RR_IN2, spd, true);
}

void pivotRight(int spd) {
  // Left wheels forward, right wheels backward
  motorSet(FL_EN, FL_IN1, FL_IN2, spd, true);
  motorSet(RL_EN, RL_IN1, RL_IN2, spd, true);
  motorSet(FR_EN, FR_IN1, FR_IN2, spd, false);
  motorSet(RR_EN, RR_IN1, RR_IN2, spd, false);
}

void stopAll() {
  motorBrake(FL_EN, FL_IN1, FL_IN2);
  motorBrake(RL_EN, RL_IN1, RL_IN2);
  motorBrake(FR_EN, FR_IN1, FR_IN2);
  motorBrake(RR_EN, RR_IN1, RR_IN2);
}

// Mecanum strafe (if mecanum wheels fitted)
void strafeLeft(int spd) {
  motorSet(FL_EN, FL_IN1, FL_IN2, spd, false);  // FL backward
  motorSet(RL_EN, RL_IN1, RL_IN2, spd, true);   // RL forward
  motorSet(FR_EN, FR_IN1, FR_IN2, spd, true);   // FR forward
  motorSet(RR_EN, RR_IN1, RR_IN2, spd, false);  // RR backward
}

void strafeRight(int spd) {
  motorSet(FL_EN, FL_IN1, FL_IN2, spd, true);
  motorSet(RL_EN, RL_IN1, RL_IN2, spd, false);
  motorSet(FR_EN, FR_IN1, FR_IN2, spd, false);
  motorSet(RR_EN, RR_IN1, RR_IN2, spd, true);
}

// ── Ramp helper ───────────────────────────────────────────
// Smooth acceleration to avoid tip-over during arm carry
void rampTo(char dir, int targetSpd, int steps = 20) {
  int fromSpd = currentSpd;
  for (int i = 1; i <= steps; i++) {
    int spd = fromSpd + (targetSpd - fromSpd) * i / steps;
    spd = constrain(spd, MIN_PWM, MAX_PWM);
    switch (dir) {
      case 'F': driveForward(spd);  break;
      case 'B': driveBackward(spd); break;
      case 'L': pivotLeft(spd);     break;
      case 'R': pivotRight(spd);    break;
    }
    delay(10);
  }
  currentDir = dir;
  currentSpd = targetSpd;
}

// ── Command parser ────────────────────────────────────────
void parseCommand(char* cmd) {
  if (cmd[0] != 'M') return;
  char dir = cmd[2];
  int  spd = constrain(atoi(cmd + 4), MIN_PWM, MAX_PWM);

  Serial.print("Drive: "); Serial.print(dir);
  Serial.print(" spd=");   Serial.println(spd);

  switch (dir) {
    case 'F': rampTo('F', spd); break;
    case 'B': rampTo('B', spd); break;
    case 'L': pivotLeft(TURN_PWM);  currentDir='L'; break;
    case 'R': pivotRight(TURN_PWM); currentDir='R'; break;
    case 'X': strafeLeft(spd);  break;
    case 'Y': strafeRight(spd); break;
    case 'S': stopAll(); currentDir='S'; currentSpd=0; break;
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        parseCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < 30) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);  // Receives from Master board

  int pins[] = {FL_EN, FL_IN1, FL_IN2, RL_EN, RL_IN1, RL_IN2,
                FR_EN, FR_IN1, FR_IN2, RR_EN, RR_IN1, RR_IN2};
  for (int p : pins) pinMode(p, OUTPUT);

  stopAll();
  Serial.println("=== Mark 1.6 Chassis Ready ===");
}

// ── Main loop ─────────────────────────────────────────────
void loop() {
  readSerial();
}
