/*
 * Project Group 4 SEMI - Dual-Axis Solar Tracker
 * Hardware: Arduino Uno R3, 2 Servos (LR = Continuous, UD = Standard), 4 LDRs
 * Features: Auto Sweep (LR) + Light Track (UD), Manual Mode, Excel Data Logging
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
#define TOLERANCE 10    
#define NIGHT_LIMIT 8   
#define LOAD_RESISTANCE 10.0 

Servo servoLR; // Continuous Rotation Servo
Servo servoUD; // Standard Servo

// --- Variables ---
bool isAutoMode = true;      
bool manualControlLR = true; 

int posUD = 45; // Initial position for UD

// Variables for Continuous Servo Timing
unsigned long lastSweepTime = 0;
bool spinningClockwise = true;
const long sweepInterval = 1800; // 2 Seconds

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
  servoLR.write(90); // 90 = STOP for continuous servo
  servoUD.write(posUD);
  
  delay(500);
}

void loop() {
  // --- 1. Check Mode Switch Button ---
  int btnModeState = digitalRead(BTN_MODE_PIN);
  if (btnModeState == LOW && lastBtnModeState == HIGH) {
    isAutoMode = !isAutoMode; 
    
    // Safety: Stop continuous servo immediately when switching modes
    servoLR.write(90); 
    
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
  
  delay(50); // Reduced delay for better responsiveness
}

// --- Automatic Mode Logic ---
void runAutomaticMode() {
  // ------------------------------------------
  // PART A: Left-Right (Azimuth) - TIME SWEEP
  // ------------------------------------------
  unsigned long currentMillis = millis();

  // Check if 2 seconds have passed
  if (currentMillis - lastSweepTime >= sweepInterval) {
    lastSweepTime = currentMillis;       // Save time
    spinningClockwise = !spinningClockwise; // Toggle direction
  }

  // NOTE: For Continuous Servos:
  // 0   = Full speed Clockwise
  // 90  = Stop
  // 180 = Full speed Counter-Clockwise
  // (Adjust 0/180 or speed if direction is reversed for your wiring)
  
  if (spinningClockwise) {
    servoLR.write(111); // Rotate CW
  } else {
    servoLR.write(73);   // Rotate CCW
  }

  // ------------------------------------------
  // PART B: Up-Down (Elevation) - LIGHT TRACKING (Unchanged)
  // ------------------------------------------
  int tl = analogRead(LDR_TL_PIN);
  int tr = analogRead(LDR_TR_PIN);
  int bl = analogRead(LDR_BL_PIN);
  int br = analogRead(LDR_BR_PIN);

  int avgTop = (tl + tr) / 2;
  int avgBot = (bl + br) / 2;
  int avgSum = (avgTop + avgBot + (tl + bl)/2 + (tr + br)/2) / 4;

  // Night Mode Check
  if (avgSum < NIGHT_LIMIT) {
    servoUD.write(30); // Return to horizon
    // We do NOT stop LR sweep based on night mode based on your request, 
    // but if you want it to stop at night, add logic here.
    return;
  }

  int diffElev = avgTop - avgBot;

  if (abs(diffElev) > TOLERANCE) {
    if (diffElev > 0) {
      posUD = posUD + 1; // Move Up
    } else {
      posUD = posUD - 1; // Move Down
    }
  }
  
  posUD = constrain(posUD, 0, 180);
  servoUD.write(posUD);
}

// --- Manual Mode Logic ---
void runManualMode() {
  int btnAxisState = digitalRead(BTN_AXIS_PIN);
  if (btnAxisState == LOW && lastBtnAxisState == HIGH) {
    manualControlLR = !manualControlLR; 
    // Stop continuous servo when switching control to other axis
    servoLR.write(90); 
    delay(200);
  }
  lastBtnAxisState = btnAxisState;

  int potVal = analogRead(POT_PIN);
  int angle = map(potVal, 0, 1023, 0, 180);

  if (manualControlLR) {
    // For Continuous Servo: 
    // angle < 90 (CW), angle 90 (Stop), angle > 90 (CCW)
    servoLR.write(angle); 
  } else {
    // For Standard Servo: Moves to position
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