/*
 * Project Group 4 SEMI - Dual-Axis Solar Tracker
 * Hardware: Arduino Uno R3, 2 Servos (LR = Continuous, UD = Standard), 4 LDRs
 * Features: Automatic Mode (Flowchart-based), Manual Mode, Excel Data Logging
 */

#include <Servo.h>

// --- Pin Definitions ---
#define LDR_TL_PIN A0  // Top-Left LDR
#define LDR_TR_PIN A1  // Top-Right LDR
#define LDR_BL_PIN A2  // Bottom-Left LDR
#define LDR_BR_PIN A3  // Bottom-Right LDR

#define POT_PIN A4     // Potentiometer for Manual Control
#define VOLT_PIN A5    // PV Panel Voltage Divider Input

#define SERVO_LR_PIN 6 // Left-Right Servo (Continuous Rotation)
#define SERVO_UD_PIN 5 // Up-Down Servo (Standard 180)

#define BTN_MODE_PIN 12 // Switch between Auto/Manual
#define BTN_AXIS_PIN 11 // Switch between LR/UD Servo in Manual

// --- Constants & Configuration ---
#define TOLERANCE 10    // Value from flowchart
#define NIGHT_LIMIT 8   // Value from flowchart
#define LOAD_RESISTANCE 10.0 

// Speeds for Continuous Servo (90 is Stop)
#define LR_SPEED_RIGHT 80 // Move Right (Adjust for speed)
#define LR_SPEED_LEFT 100 // Move Left (Adjust for speed)
#define LR_STOP 90

Servo servoLR; // Continuous Rotation Servo
Servo servoUD; // Standard Servo

// --- Variables ---
bool isAutoMode = true;      
bool manualControlLR = true; 

int posUD = 45; // Initial position for UD

// Button State Tracking
int lastBtnModeState = HIGH;
int lastBtnAxisState = HIGH;

void setup() {
  Serial.begin(9600);
  Serial.println("CLEARDATA"); 
  Serial.println("LABEL,Time,Mode,Voltage(V),Current(mA),Power(mW)"); 

  servoLR.attach(SERVO_LR_PIN);
  servoUD.attach(SERVO_UD_PIN);
  
  pinMode(BTN_MODE_PIN, INPUT_PULLUP);
  pinMode(BTN_AXIS_PIN, INPUT_PULLUP);
  
  // Initial positions
  servoLR.write(LR_STOP); 
  servoUD.write(posUD);
  
  delay(500);
}

void loop() {
  // --- 1. Check Mode Switch Button ---
  int btnModeState = digitalRead(BTN_MODE_PIN);
  if (btnModeState == LOW && lastBtnModeState == HIGH) {
    isAutoMode = !isAutoMode; 
    servoLR.write(LR_STOP); // Safety stop
    delay(200); 
  }
  lastBtnModeState = btnModeState;

  // --- 2. Execute Mode Logic ---
  if (isAutoMode) {
    runAutomaticMode();
  } else {
    runManualMode();
  }

  // --- 3. Data Logging to Excel ---
  logDataToExcel();
  
  delay(50); 
}

// --- Automatic Mode Logic based on Flowchart ---
void runAutomaticMode() {
  // 1. Read the analog value from each LDR sensor
  int topleft = analogRead(LDR_TL_PIN);
  int topright = analogRead(LDR_TR_PIN);
  int botleft = analogRead(LDR_BL_PIN);
  int botright = analogRead(LDR_BR_PIN);

  // 2. Calculate the average values
  int avgtop = (topright + topleft) / 2;
  int avgbot = (botright + botleft) / 2;
  int avgright = (topright + botright) / 2;
  int avgleft = (topleft + botleft) / 2;

  // 3. Calculate the differences (Azimuth & elevation)
  int diffelev = avgtop - avgbot;
  int diffazi = avgright - avgleft;
  int avgsum = (avgtop + avgbot + avgright + avgleft) / 4;

  // 4. Check Night Mode (avgsum < 8)
  if (avgsum < NIGHT_LIMIT) {
    // "Rotate the Servo Motors to the initial position"
    servoLR.write(LR_STOP); // Stop LR servo
    posUD = 30;             // Set UD to sunrise/horizon position
    servoUD.write(posUD);
    return; // Go back to Start
  }

  // 5. Azimuth (Left-Right) Control
  // Check |diffazi| <= 10
  if (abs(diffazi) <= TOLERANCE) {
    // "Stop the Left-Right Servo Motor"
    servoLR.write(LR_STOP);
  } 
  else {
    // Check diffazi > 0
    if (diffazi > 0) {
      // "Left-Right Servo Motor move PV panel Right"
      servoLR.write(LR_SPEED_RIGHT); 
    } else {
      // "Left-Right Servo Motor move PV panel Left"
      servoLR.write(LR_SPEED_LEFT);
    }
  }

  // 6. Elevation (Up-Down) Control
  // Check |diffelev| <= 10
  if (abs(diffelev) <= TOLERANCE) {
    // "Stop the Up-Down Servo Motor"
    // Do nothing (position doesn't change)
  } 
  else {
    // Check diffelev > 0
    if (diffelev > 0) {
      // "Up-Down Servo Motor move PV panel Up"
      posUD = posUD + 1;
    } else {
      // "Up-Down Servo Motor move PV panel Down"
      posUD = posUD - 1;
    }
    // Write the new position for the standard servo
    posUD = constrain(posUD, 0, 180);
    servoUD.write(posUD);
  }
  // The logic then goes back to Start
}

// --- Manual Mode Logic ---
void runManualMode() {
  int btnAxisState = digitalRead(BTN_AXIS_PIN);
  if (btnAxisState == LOW && lastBtnAxisState == HIGH) {
    manualControlLR = !manualControlLR; 
    servoLR.write(LR_STOP); // Stop when switching axis
    delay(200);
  }
  lastBtnAxisState = btnAxisState;

  int potVal = analogRead(POT_PIN);
  int angle = map(potVal, 0, 1023, 0, 180);

  if (manualControlLR) {
    // Potentiometer controls speed/direction of continuous servo
    servoLR.write(angle); 
  } else {
    // Potentiometer controls position of standard servo
    servoUD.write(angle);
    posUD = angle;
  }
}

// --- Data Logging Logic ---
void logDataToExcel() {
  float sensorValue = analogRead(VOLT_PIN);
  float voltage = sensorValue * (5.0 / 1023.0); 
  float current = (voltage / LOAD_RESISTANCE) * 1000; 
  float power = voltage * current; 

  Serial.print("DATA,TIME,");
  if (isAutoMode) Serial.print("Auto");
  else Serial.print("Manual");
  Serial.print(",");
  Serial.print(voltage);
  Serial.print(",");
  Serial.print(current);
  Serial.print(",");
  Serial.println(power);
}