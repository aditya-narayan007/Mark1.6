/*
 * =========================================================
 *  Mark 1.6 — Master Controller
 *  Arduino Mega (recommended) or Uno
 * =========================================================
 *  Receives commands over Bluetooth (HC-05) or NRF24L01+
 *  Parses the command packet and dispatches to:
 *    - 4WD Chassis  (via Serial1 / software serial)
 *    - Robotic Arm  (servo control, direct on this board)
 *
 *  Command Protocol (ASCII, newline-terminated):
 *    Movement : M <dir> <speed>     e.g. "M F 180"
 *    Arm      : A <joint> <angle>   e.g. "A 2 90"
 *    Gripper  : G <0|1>             e.g. "G 1"  (1=close)
 *    Stop     : S
 *    Preset   : P <id>              e.g. "P 1"  (home)
 *
 *  Wireless selection: define WIRELESS_BT or WIRELESS_NRF
 * =========================================================
 */

#define WIRELESS_BT          // Use Bluetooth (HC-05). Change to WIRELESS_NRF for NRF24L01+

#include <Servo.h>

#ifdef WIRELESS_NRF
  #include <SPI.h>
  #include <RF24.h>
  RF24 radio(9, 10);          // CE=9, CSN=10
  const byte RX_ADDR[6] = "MK16A";
#endif

// ── Serial ports ─────────────────────────────────────────
// Mega: Serial=USB, Serial1=BT, Serial2=Chassis UART
// Uno:  use SoftwareSerial for both (define below)
#ifdef WIRELESS_BT
  #define BT_SERIAL   Serial1   // HC-05 TX→Mega RX1(19), RX→Mega TX1(18)
#endif
#define CHASSIS_SERIAL  Serial2 // To chassis Arduino: TX2(16)→chassisRX

// ── Arm Servos ────────────────────────────────────────────
// 5-DOF arm: Base, Shoulder, Elbow, Wrist, Gripper
const int SERVO_PINS[5] = {2, 3, 4, 5, 6};
Servo armServos[5];

// Joint angle limits (degrees)
const int ANGLE_MIN[5] = {  0,  20,  10,   0,   0};
const int ANGLE_MAX[5] = {180, 160, 170, 180,  90};

// Current angles
int currentAngle[5] = {90, 90, 90, 90, 45};

// ── Preset positions ──────────────────────────────────────
// Preset 0=Home, 1=Pick, 2=Place, 3=Carry, 4=Stow
const int PRESETS[5][5] = {
  {90,  90,  90,  90, 45},   // 0 Home
  {90, 140, 140,  90,  0},   // 1 Pick (reach down, gripper open)
  {45,  60,  60,  90, 90},   // 2 Place (reach left, gripper closed)
  {90,  70,  90,  90, 90},   // 3 Carry (arm raised, gripper closed)
  {90, 160, 160,  45, 90},   // 4 Stow  (arm folded back)
};

// ── Command buffer ────────────────────────────────────────
char cmdBuf[64];
uint8_t cmdLen = 0;

// ── NRF packet ────────────────────────────────────────────
#ifdef WIRELESS_NRF
char nrfBuf[32];
#endif

// ── Utilities ─────────────────────────────────────────────
void moveServo(int joint, int angle) {
  angle = constrain(angle, ANGLE_MIN[joint], ANGLE_MAX[joint]);
  currentAngle[joint] = angle;
  armServos[joint].write(angle);
}

void sweepServo(int joint, int target, int stepDelay = 12) {
  int from = currentAngle[joint];
  target   = constrain(target, ANGLE_MIN[joint], ANGLE_MAX[joint]);
  int step = (target > from) ? 1 : -1;
  for (int a = from; a != target; a += step) {
    armServos[joint].write(a);
    delay(stepDelay);
  }
  currentAngle[joint] = target;
}

void runPreset(int id) {
  if (id < 0 || id > 4) return;
  // Move all joints simultaneously in small steps
  int steps = 40;
  for (int s = 0; s <= steps; s++) {
    for (int j = 0; j < 5; j++) {
      int from = currentAngle[j];
      int to   = PRESETS[id][j];
      int angle = from + (int)((float)(to - from) * s / steps);
      armServos[j].write(angle);
    }
    delay(15);
  }
  for (int j = 0; j < 5; j++) currentAngle[j] = PRESETS[id][j];
  Serial.print("Preset "); Serial.print(id); Serial.println(" done");
}

// Send drive command to chassis Arduino via UART
void sendChassisCmd(const char* cmd) {
  CHASSIS_SERIAL.println(cmd);
}

// ── Command parser ────────────────────────────────────────
void parseCommand(char* cmd) {
  Serial.print("CMD: "); Serial.println(cmd);

  switch (cmd[0]) {

    case 'M': {   // Movement: M <F|B|L|R|S> <speed>
      char dir = cmd[2];
      int  spd = atoi(cmd + 4);
      char buf[16];
      snprintf(buf, sizeof(buf), "M %c %d", dir, spd);
      sendChassisCmd(buf);
      break;
    }

    case 'A': {   // Arm joint: A <joint 0-4> <angle>
      int joint = atoi(cmd + 2);
      int angle = atoi(cmd + 4);
      if (joint >= 0 && joint < 5) sweepServo(joint, angle);
      break;
    }

    case 'G': {   // Gripper: G <0|1>
      int close = atoi(cmd + 2);
      sweepServo(4, close ? ANGLE_MAX[4] : ANGLE_MIN[4]);
      break;
    }

    case 'S': {   // Stop all
      sendChassisCmd("M S 0");
      break;
    }

    case 'P': {   // Preset
      int id = atoi(cmd + 2);
      runPreset(id);
      break;
    }

    default:
      Serial.println("Unknown command");
      break;
  }
}

// ── Read BT/USB serial ────────────────────────────────────
void readSerial(Stream& port) {
  while (port.available()) {
    char c = port.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        parseCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < 62) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

#ifdef WIRELESS_BT
  BT_SERIAL.begin(9600);    // Match HC-05 baud rate
#endif

#ifdef WIRELESS_NRF
  radio.begin();
  radio.openReadingPipe(0, RX_ADDR);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
#endif

  CHASSIS_SERIAL.begin(9600);

  // Attach and initialise servos
  for (int j = 0; j < 5; j++) {
    armServos[j].attach(SERVO_PINS[j]);
    armServos[j].write(currentAngle[j]);
  }
  delay(500);

  runPreset(0);   // Go to home on boot
  Serial.println("=== Mark 1.6 Master Ready ===");
}

// ── Main loop ─────────────────────────────────────────────
void loop() {
  // Read from USB monitor (for manual testing)
  readSerial(Serial);

#ifdef WIRELESS_BT
  readSerial(BT_SERIAL);
#endif

#ifdef WIRELESS_NRF
  if (radio.available()) {
    radio.read(nrfBuf, sizeof(nrfBuf));
    // Copy into cmdBuf and parse
    strncpy(cmdBuf, nrfBuf, 62);
    cmdBuf[62] = '\0';
    parseCommand(cmdBuf);
  }
#endif
}
