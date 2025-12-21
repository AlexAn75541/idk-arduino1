#include <Servo.h>

// ================== PIN CONFIG ==================
#define LDR_TR A0
#define LDR_TL A1
#define LDR_BR A2
#define LDR_BL A3

#define POT_PIN A4
#define VOLT_PIN A5

#define SERVO_EL_PIN 5     // 180° servo (up-down)
#define SERVO_AZ_PIN 6     // 360° servo (right-left)

#define BTN_MODE 12
#define BTN_AXIS 13

// ================== VARIABLES ==================
Servo servoEL;   // Elevation (180°)
Servo servoAZ;   // Azimuth (360°)

bool autoMode = false;
bool axisSelect = false;   // false = AZ, true = EL

int threshold = 30;

// Servo 360 tuning
int azStop = 90;
int azSpeed = 8;  // rotation speed (±)

// ================== SETUP ==================
void setup() {
  Serial.begin(9600);

  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_AXIS, INPUT_PULLUP);

  servoEL.attach(SERVO_EL_PIN);
  servoAZ.attach(SERVO_AZ_PIN);

  // Initial positions
  servoEL.write(90);      // middle
  servoAZ.write(azStop); // stop
  delay(500);

  Serial.println("Solar Tracker Ready");
}

// ================== LOOP ==================
void loop() {
  handleModeButton();

  if (autoMode) {
    automaticTracker();
    Serial.println("Mode: AUTO");
  } else {
    manualTracker();
    Serial.println("Mode: MANUAL");
  }

  delay(20);
}

// ================== BUTTON HANDLING ==================
void handleModeButton() {
  static bool lastState = HIGH;
  bool current = digitalRead(BTN_MODE);

  if (lastState == HIGH && current == LOW) {
    autoMode = !autoMode;
    delay(200); // debounce
  }
  lastState = current;
}

// ================== AUTOMATIC TRACKER ==================
void automaticTracker() {
  int tr = analogRead(LDR_TR);
  int tl = analogRead(LDR_TL);
  int br = analogRead(LDR_BR);
  int bl = analogRead(LDR_BL);

  int avgTop = (tr + tl) / 2;
  int avgBot = (br + bl) / 2;
  int avgLeft = (tl + bl) / 2;
  int avgRight = (tr + br) / 2;

  int diffEL = avgTop - avgBot;
  int diffAZ = avgRight - avgLeft;

  // ===== AZIMUTH (360 SERVO) =====
  if (abs(diffAZ) > threshold) {
    if (diffAZ > 0) {
      servoAZ.write(azStop + azSpeed); // turn right
    } else {
      servoAZ.write(azStop - azSpeed); // ____ left
    }
  } else {
    servoAZ.write(azStop); // stop
  }

  // ===== ELEVATION (180 SERVO) =====
  static int elPos = 90;

  if (abs(diffEL) > threshold) {
    if (diffEL > 0 && elPos < 175) elPos += 1;
    if (diffEL < 0 && elPos > 5)   elPos -= 1;
    servoEL.write(elPos);
  }
}

// ================== MANUAL TRACKER ==================
void manualTracker() {
  static bool lastAxisBtn = HIGH;
  bool current = digitalRead(BTN_AXIS);

  if (lastAxisBtn == HIGH && current == LOW) {
    axisSelect = !axisSelect;
    delay(200);
  }
  lastAxisBtn = current;

  int pot = analogRead(POT_PIN);

  if (!axisSelect) {
    // AZ manual (360 servo)
    int speed = map(pot, 0, 1023, -15, 15);
    servoAZ.write(azStop + speed);
  } else {
    // EL manual (180 servo)
    int pos = map(pot, 0, 1023, 0, 180);
    servoEL.write(pos);
  }
}