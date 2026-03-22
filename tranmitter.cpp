/*
 * =========================================================
 *  Mark 1.6 — NRF24L01+ Handheld Transmitter
 *  Arduino Uno + NRF24L01+ + Joysticks + Buttons
 * =========================================================
 *  This sketch runs on the CONTROLLER side (handheld unit).
 *  It reads two analog joysticks and a set of buttons,
 *  then packages commands and transmits via NRF24L01+.
 *
 *  Hardware layout:
 *    Left joystick  → 4WD drive (X=turn, Y=forward/back)
 *    Right joystick → Arm shoulder(Y) + base rotation(X)
 *    Potentiometer  → Elbow angle
 *    Buttons        → Presets 0–4, gripper toggle
 *
 *  Wiring:
 *    NRF24: CE=9, CSN=10, MOSI=11, MISO=12, SCK=13
 *    Left JY X=A0, Y=A1
 *    Right JY X=A2, Y=A3
 *    Elbow pot=A4
 *    Gripper BTN=2, Preset BTNs=3,4,5,6,7
 * =========================================================
 */

#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);
const byte TX_ADDR[6] = "MK16A";

// ── Analog inputs ─────────────────────────────────────────
const int JL_X   = A0;   // Left joystick X  → turning
const int JL_Y   = A1;   // Left joystick Y  → forward/back
const int JR_X   = A2;   // Right joystick X → arm base rotation
const int JR_Y   = A3;   // Right joystick Y → arm shoulder
const int POT_EL = A4;   // Potentiometer    → elbow

// ── Digital buttons ───────────────────────────────────────
const int BTN_GRIP    = 2;
const int BTN_P0      = 3;  // Home
const int BTN_P1      = 4;  // Pick
const int BTN_P2      = 5;  // Place
const int BTN_P3      = 6;  // Carry
const int BTN_P4      = 7;  // Stow

// ── Deadzone ──────────────────────────────────────────────
const int DEADZONE    = 50;   // ADC units around center (512)
const int CENTER      = 512;

// ── State ─────────────────────────────────────────────────
bool   gripperClosed  = false;
bool   lastGripBtn    = HIGH;
char   lastMoveDir    = 'S';
int    lastMoveSpd    = 0;
int    lastArmAngles[5] = {90, 90, 90, 90, 45};
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 50;  // ms (20 Hz)

char packet[32];

// ── Helpers ───────────────────────────────────────────────
int mapJoy(int raw, int minOut, int maxOut) {
  // Returns 0 in deadzone, else mapped value
  if (abs(raw - CENTER) < DEADZONE) return 0;
  if (raw > CENTER) return map(raw, CENTER + DEADZONE, 1023, 0, maxOut);
  return map(raw, 0, CENTER - DEADZONE, minOut, 0);
}

void sendPacket(const char* cmd) {
  memset(packet, 0, sizeof(packet));
  strncpy(packet, cmd, 31);
  radio.write(packet, sizeof(packet));
  Serial.println(packet);
}

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // Button pins with pull-up
  int btns[] = {BTN_GRIP, BTN_P0, BTN_P1, BTN_P2, BTN_P3, BTN_P4};
  for (int b : btns) pinMode(b, INPUT_PULLUP);

  radio.begin();
  radio.openWritingPipe(TX_ADDR);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  Serial.println("=== Mark 1.6 Transmitter Ready ===");
}

// ── Main loop ─────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── Preset buttons (send once on press) ──────────────
  if (!digitalRead(BTN_P0)) { sendPacket("P 0"); delay(200); }
  if (!digitalRead(BTN_P1)) { sendPacket("P 1"); delay(200); }
  if (!digitalRead(BTN_P2)) { sendPacket("P 2"); delay(200); }
  if (!digitalRead(BTN_P3)) { sendPacket("P 3"); delay(200); }
  if (!digitalRead(BTN_P4)) { sendPacket("P 4"); delay(200); }

  // ── Gripper toggle ────────────────────────────────────
  bool gripBtn = digitalRead(BTN_GRIP);
  if (gripBtn == LOW && lastGripBtn == HIGH) {
    gripperClosed = !gripperClosed;
    snprintf(packet, sizeof(packet), "G %d", gripperClosed ? 1 : 0);
    sendPacket(packet);
    delay(50);
  }
  lastGripBtn = gripBtn;

  // ── Throttled axis updates ────────────────────────────
  if (now - lastSend < SEND_INTERVAL) return;
  lastSend = now;

  // ── Drive (Left joystick) ────────────────────────────
  int jlX = analogRead(JL_X);
  int jlY = analogRead(JL_Y);
  int fwdVal  = mapJoy(jlY, -220, 220);
  int turnVal = mapJoy(jlX, -220, 220);

  char newDir = 'S';
  int  newSpd = 0;

  if (abs(fwdVal) > abs(turnVal)) {
    newDir = (fwdVal > 0) ? 'F' : 'B';
    newSpd = abs(fwdVal);
  } else if (abs(turnVal) > 0) {
    newDir = (turnVal > 0) ? 'R' : 'L';
    newSpd = abs(turnVal);
  }

  if (newDir != lastMoveDir || abs(newSpd - lastMoveSpd) > 10) {
    snprintf(packet, sizeof(packet), "M %c %d", newDir, newSpd);
    sendPacket(packet);
    lastMoveDir = newDir;
    lastMoveSpd = newSpd;
  }

  // ── Arm base (Right JY X) ────────────────────────────
  int baseVal = mapJoy(analogRead(JR_X), -3, 3);
  if (baseVal != 0) {
    int newAngle = constrain(lastArmAngles[0] + baseVal, 0, 180);
    if (newAngle != lastArmAngles[0]) {
      snprintf(packet, sizeof(packet), "A 0 %d", newAngle);
      sendPacket(packet);
      lastArmAngles[0] = newAngle;
    }
  }

  // ── Shoulder (Right JY Y) ────────────────────────────
  int shoulderVal = mapJoy(analogRead(JR_Y), -3, 3);
  if (shoulderVal != 0) {
    int newAngle = constrain(lastArmAngles[1] + shoulderVal, 20, 160);
    if (newAngle != lastArmAngles[1]) {
      snprintf(packet, sizeof(packet), "A 1 %d", newAngle);
      sendPacket(packet);
      lastArmAngles[1] = newAngle;
    }
  }

  // ── Elbow (Potentiometer) ─────────────────────────────
  int elbowAngle = map(analogRead(POT_EL), 0, 1023, 10, 170);
  if (abs(elbowAngle - lastArmAngles[2]) > 3) {
    snprintf(packet, sizeof(packet), "A 2 %d", elbowAngle);
    sendPacket(packet);
    lastArmAngles[2] = elbowAngle;
  }
}
