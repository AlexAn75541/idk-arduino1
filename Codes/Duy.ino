/*
 * ========================================================================
 * DUAL-AXIS SOLAR TRACKER WITH ARDUINO UNO R3
 * ========================================================================
 * 
 * PROJECT DESCRIPTION: 
 * This project implements an automatic/manual dual-axis solar tracker
 * that can rotate to track the sun using LDR sensors (automatic mode)
 * or be controlled manually using a potentiometer. 
 * 
 * HARDWARE COMPONENTS:
 * - Arduino Uno R3
 * - 4x LDR sensors (Light Dependent Resistors)
 * - 2x Servo motors (MG996R for left-right, SG90 for up-down)
 * - 1x Potentiometer (for manual control)
 * - 2x Push buttons (mode switching and motor selection)
 * - 1x Solar PV panel with voltage sensor
 * - Resistors:  330Ω for LDR voltage dividers, load resistor for current measurement
 * 
 * FEATURES:
 * - Automatic sun tracking using 4 LDR sensors
 * - Manual control using potentiometer
 * - Real-time data logging to Excel via PLX-DAQ
 * - Voltage, current, and power monitoring
 * - Night mode:  returns to sunrise position
 * 
 * AUTHOR:  ducduyhsme
 * DATE: 2025-12-13
 * ========================================================================
 */

// Include the Servo library for controlling servo motors
#include <Servo.h>

// ========================================================================
// PIN DEFINITIONS
// ========================================================================

// LDR Sensor Pins (Analog Inputs)
#define LDR_TOP_LEFT     A0  // Top-left LDR sensor
#define LDR_TOP_RIGHT    A1  // Top-right LDR sensor
#define LDR_BOTTOM_LEFT  A2  // Bottom-left LDR sensor
#define LDR_BOTTOM_RIGHT A3  // Bottom-right LDR sensor

// Control Input Pins
#define POT_PIN          A4  // Potentiometer for manual control
#define PV_VOLTAGE_PIN   A5  // PV panel voltage measurement

// Servo Motor Control Pins (PWM)
#define SERVO_UP_DOWN    5   // Up-down servo motor (SG90) - controls elevation (south to north)
#define SERVO_LEFT_RIGHT 6   // Left-right servo motor (MG996R) - controls azimuth (east to west)

// Push Button Pins (Digital Inputs)
#define BUTTON_MOTOR_SELECT 11  // Button to select which motor to control in manual mode
#define BUTTON_MODE_SWITCH  12  // Button to switch between automatic and manual modes

// ========================================================================
// GLOBAL CONSTANTS
// ========================================================================

// Servo Motor Position Limits
#define SERVO_MIN_ANGLE  0    // Minimum servo angle (degrees)
#define SERVO_MAX_ANGLE  180  // Maximum servo angle (degrees)

// Light Sensing Thresholds
#define LIGHT_TOLERANCE  10   // Acceptable difference between LDR readings for stability
#define NIGHT_THRESHOLD  8    // Average LDR value below which system enters night mode

// Sunrise Position (Default position when night is detected)
#define SUNRISE_HORIZONTAL 0   // Left-right servo angle at sunrise (east position)
#define SUNRISE_VERTICAL   30  // Up-down servo angle at sunrise

// PV Panel Measurement Constants
#define LOAD_RESISTOR    10.0  // Load resistor value in Ohms (adjust to your actual value)
#define VOLTAGE_DIVIDER  5.0   // Voltage divider ratio (adjust based on your circuit)

// Timing Constants
#define SERIAL_BAUD_RATE 9600  // Serial communication speed for PLX-DAQ
#define LOOP_DELAY       100   // Main loop delay in milliseconds

// ========================================================================
// GLOBAL VARIABLES
// ========================================================================

// Create servo motor objects
Servo servoUpDown;      // Servo for vertical movement (elevation)
Servo servoLeftRight;   // Servo for horizontal movement (azimuth)

// Mode Control Variables
bool isAutoMode = true;        // true = automatic mode, false = manual mode
bool controlUpDown = true;     // true = control up-down motor, false = control left-right motor

// Button State Variables (for debouncing)
bool lastModeButtonState = HIGH;      // Previous state of mode switch button
bool lastMotorSelectButtonState = HIGH; // Previous state of motor select button
unsigned long lastModeDebounceTime = 0;   // Last time mode button was toggled
unsigned long lastMotorDebounceTime = 0;  // Last time motor select button was toggled
const unsigned long debounceDelay = 50;   // Debounce delay in milliseconds

// LDR Sensor Values
int ldrTopLeft = 0;      // Reading from top-left LDR
int ldrTopRight = 0;     // Reading from top-right LDR
int ldrBottomLeft = 0;   // Reading from bottom-left LDR
int ldrBottomRight = 0;  // Reading from bottom-right LDR

// Servo Position Variables
int currentHorizontalAngle = SUNRISE_HORIZONTAL;  // Current left-right servo angle
int currentVerticalAngle = SUNRISE_VERTICAL;      // Current up-down servo angle

// PV Panel Measurement Variables
float pvVoltage = 0.0;   // PV panel voltage in volts
float pvCurrent = 0.0;   // PV panel current in amperes
float pvPower = 0.0;     // PV panel power in watts

// ========================================================================
// SETUP FUNCTION - Runs once at startup
// ========================================================================
void setup() {
  // Initialize serial communication for data logging to Excel via PLX-DAQ
  Serial.begin(SERIAL_BAUD_RATE);
  
  // Send PLX-DAQ initialization commands
  Serial.println("CLEARDATA");  // Clear any existing data in Excel
  Serial.println("LABEL,Time,Mode,Voltage(V),Current(A),Power(W),Horizontal Angle,Vertical Angle");
  // Create column headers:  Time, Mode, Voltage, Current, Power, Servo Angles
  
  // Attach servo motors to their respective pins
  servoUpDown.attach(SERVO_UP_DOWN);        // Initialize up-down servo on pin 5
  servoLeftRight.attach(SERVO_LEFT_RIGHT);  // Initialize left-right servo on pin 6
  
  // Set servos to initial sunrise position
  servoLeftRight.write(currentHorizontalAngle);  // Move to east position
  servoUpDown.write(currentVerticalAngle);       // Set initial elevation
  
  // Configure button pins as inputs with internal pull-up resistors
  // Pull-up resistors make the pins HIGH when button is not pressed
  pinMode(BUTTON_MODE_SWITCH, INPUT_PULLUP);    // Mode switch button
  pinMode(BUTTON_MOTOR_SELECT, INPUT_PULLUP);   // Motor select button
  
  // Wait for servos to reach initial position
  delay(1000);
  
  // Send startup confirmation message
  Serial.println("Solar Tracker Initialized - Automatic Mode Active");
}

// ========================================================================
// MAIN LOOP FUNCTION - Runs repeatedly
// ========================================================================
void loop() {
  // Check if mode switch button is pressed (with debouncing)
  checkModeButton();
  
  // Execute appropriate control logic based on current mode
  if (isAutoMode) {
    // AUTOMATIC MODE: Track sun using LDR sensors
    automaticMode();
  } else {
    // MANUAL MODE: Control servos using potentiometer
    manualMode();
  }
  
  // Read PV panel voltage, calculate current and power
  measurePVPanel();
  
  // Send data to Excel for real-time visualization
  sendDataToExcel();
  
  // Small delay to stabilize readings and control loop speed
  delay(LOOP_DELAY);
}

// ========================================================================
// BUTTON HANDLING FUNCTIONS
// ========================================================================

/*
 * Function: checkModeButton
 * -------------------------
 * Checks if the mode switch button is pressed and toggles between
 * automatic and manual modes.  Implements debouncing to avoid false triggers.
 * 
 * Button connected to pin 12 with pull-up resistor: 
 * - HIGH (not pressed) = button not pressed
 * - LOW (pressed) = button is pressed
 */
void checkModeButton() {
  // Read current button state
  bool currentButtonState = digitalRead(BUTTON_MODE_SWITCH);
  
  // Check if button state has changed
  if (currentButtonState != lastModeButtonState) {
    // Reset debounce timer
    lastModeDebounceTime = millis();
  }
  
  // Check if enough time has passed to consider it a valid button press
  if ((millis() - lastModeDebounceTime) > debounceDelay) {
    // If button is pressed (LOW) and it's a new press
    if (currentButtonState == LOW && lastModeButtonState == HIGH) {
      // Toggle between automatic and manual mode
      isAutoMode = ! isAutoMode;
      
      // Send mode change notification to serial monitor/Excel
      if (isAutoMode) {
        Serial.println("Mode Changed:  AUTOMATIC");
      } else {
        Serial.println("Mode Changed: MANUAL");
      }
    }
  }
  
  // Save current button state for next iteration
  lastModeButtonState = currentButtonState;
}

/*
 * Function: checkMotorSelectButton
 * ---------------------------------
 * Checks if motor selection button is pressed in manual mode. 
 * Toggles between controlling up-down servo and left-right servo.
 * Only active in manual mode.
 */
void checkMotorSelectButton() {
  // Read current button state
  bool currentButtonState = digitalRead(BUTTON_MOTOR_SELECT);
  
  // Check if button state has changed
  if (currentButtonState != lastMotorSelectButtonState) {
    // Reset debounce timer
    lastMotorDebounceTime = millis();
  }
  
  // Check if enough time has passed to consider it a valid button press
  if ((millis() - lastMotorDebounceTime) > debounceDelay) {
    // If button is pressed (LOW) and it's a new press
    if (currentButtonState == LOW && lastMotorSelectButtonState == HIGH) {
      // Toggle between controlling up-down and left-right motors
      controlUpDown = !controlUpDown;
      
      // Send motor selection notification
      if (controlUpDown) {
        Serial.println("Manual Control: UP-DOWN Motor Selected");
      } else {
        Serial.println("Manual Control: LEFT-RIGHT Motor Selected");
      }
    }
  }
  
  // Save current button state for next iteration
  lastMotorSelectButtonState = currentButtonState;
}

// ========================================================================
// AUTOMATIC MODE FUNCTION
// ========================================================================

/*
 * Function: automaticMode
 * -----------------------
 * Implements automatic sun tracking algorithm using 4 LDR sensors. 
 * 
 * ALGORITHM:
 * 1. Read all 4 LDR sensors
 * 2. Calculate average light from left sensors vs right sensors
 * 3. Calculate average light from top sensors vs bottom sensors
 * 4. Move servos toward the brighter direction
 * 5. Stop when difference is within tolerance range
 * 6. If night detected (avg < threshold), return to sunrise position
 * 
 * LDR SENSOR POSITIONS:
 *     [Top-Left]    [Top-Right]
 *           \          /
 *            \        /
 *         [Solar Panel]
 *            /        \
 *           /          \
 *   [Bottom-Left]  [Bottom-Right]
 */
void automaticMode() {
  // Read all four LDR sensors (ADC returns 0-1023)
  ldrTopLeft = analogRead(LDR_TOP_LEFT);
  ldrTopRight = analogRead(LDR_TOP_RIGHT);
  ldrBottomLeft = analogRead(LDR_BOTTOM_LEFT);
  ldrBottomRight = analogRead(LDR_BOTTOM_RIGHT);
  
  // Calculate average light intensity from left side sensors
  int avgLeft = (ldrTopLeft + ldrBottomLeft) / 2;
  
  // Calculate average light intensity from right side sensors
  int avgRight = (ldrTopRight + ldrBottomRight) / 2;
  
  // Calculate average light intensity from top sensors
  int avgTop = (ldrTopLeft + ldrTopRight) / 2;
  
  // Calculate average light intensity from bottom sensors
  int avgBottom = (ldrBottomLeft + ldrBottomRight) / 2;
  
  // Calculate overall average light (for night detection)
  int avgTotal = (ldrTopLeft + ldrTopRight + ldrBottomLeft + ldrBottomRight) / 4;
  
  // --- NIGHT DETECTION ---
  // If average light is below threshold, it's night time
  if (avgTotal < NIGHT_THRESHOLD) {
    // Return to sunrise position (east and initial elevation)
    currentHorizontalAngle = SUNRISE_HORIZONTAL;
    currentVerticalAngle = SUNRISE_VERTICAL;
    servoLeftRight.write(currentHorizontalAngle);
    servoUpDown.write(currentVerticalAngle);
    return;  // Exit function, no tracking during night
  }
  
  // --- HORIZONTAL AXIS CONTROL (East-West / Left-Right) ---
  // Calculate difference between left and right light intensity
  int horizontalDiff = avgLeft - avgRight;
  
  if (horizontalDiff > LIGHT_TOLERANCE) {
    // Left side receives more light, move tracker LEFT (towards east in morning)
    if (currentHorizontalAngle > SERVO_MIN_ANGLE) {
      currentHorizontalAngle--;  // Decrease angle to move left
      servoLeftRight.write(currentHorizontalAngle);
    }
  } 
  else if (horizontalDiff < -LIGHT_TOLERANCE) {
    // Right side receives more light, move tracker RIGHT (towards west in afternoon)
    if (currentHorizontalAngle < SERVO_MAX_ANGLE) {
      currentHorizontalAngle++;  // Increase angle to move right
      servoLeftRight.write(currentHorizontalAngle);
    }
  }
  // If difference is within tolerance [-10, 10], tracker is aligned - no movement needed
  
  // --- VERTICAL AXIS CONTROL (South-North / Up-Down) ---
  // Calculate difference between top and bottom light intensity
  int verticalDiff = avgTop - avgBottom;
  
  if (verticalDiff > LIGHT_TOLERANCE) {
    // Top sensors receive more light, move tracker UP (towards north)
    if (currentVerticalAngle < SERVO_MAX_ANGLE) {
      currentVerticalAngle++;  // Increase angle to move up
      servoUpDown.write(currentVerticalAngle);
    }
  } 
  else if (verticalDiff < -LIGHT_TOLERANCE) {
    // Bottom sensors receive more light, move tracker DOWN (towards south)
    if (currentVerticalAngle > SERVO_MIN_ANGLE) {
      currentVerticalAngle--;  // Decrease angle to move down
      servoUpDown.write(currentVerticalAngle);
    }
  }
  // If difference is within tolerance [-10, 10], tracker is aligned - no movement needed
}

// ========================================================================
// MANUAL MODE FUNCTION
// ========================================================================

/*
 * Function:  manualMode
 * --------------------
 * Allows manual control of solar tracker using potentiometer.
 * Motor selection button (pin 11) switches between controlling: 
 * - Up-Down motor (elevation control)
 * - Left-Right motor (azimuth control)
 * 
 * Potentiometer value (0-1023) is mapped to servo angle (0-180 degrees)
 */
void manualMode() {
  // Check if motor selection button is pressed
  checkMotorSelectButton();
  
  // Read potentiometer value (0-1023 from 10-bit ADC)
  int potValue = analogRead(POT_PIN);
  
  // Map potentiometer value to servo angle range (0-180 degrees)
  // map(value, fromLow, fromHigh, toLow, toHigh)
  int targetAngle = map(potValue, 0, 1023, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  // Control the selected motor based on controlUpDown flag
  if (controlUpDown) {
    // Control UP-DOWN motor (vertical movement)
    currentVerticalAngle = targetAngle;
    servoUpDown. write(currentVerticalAngle);
  } 
  else {
    // Control LEFT-RIGHT motor (horizontal movement)
    currentHorizontalAngle = targetAngle;
    servoLeftRight.write(currentHorizontalAngle);
  }
}

// ========================================================================
// PV PANEL MEASUREMENT FUNCTION
// ========================================================================

/*
 * Function: measurePVPanel
 * ------------------------
 * Reads PV panel voltage from analog pin A5 and calculates: 
 * - Voltage:  Converted from ADC reading using voltage divider ratio
 * - Current: Calculated using Ohm's law (I = V / R) with load resistor
 * - Power: Calculated as P = V × I
 * 
 * NOTE:  Adjust VOLTAGE_DIVIDER and LOAD_RESISTOR constants based on
 * your actual circuit configuration
 */
void measurePVPanel() {
  // Read raw ADC value from voltage sensor (0-1023)
  int rawVoltage = analogRead(PV_VOLTAGE_PIN);
  
  // Convert ADC reading to actual voltage
  // Formula: V = (ADC_value / 1023) × Reference_Voltage × Voltage_Divider_Ratio
  pvVoltage = (rawVoltage / 1023.0) * 5.0 * VOLTAGE_DIVIDER;
  
  // Calculate current using Ohm's Law: I = V / R
  // Where R is the load resistor value
  pvCurrent = pvVoltage / LOAD_RESISTOR;
  
  // Calculate power:  P = V × I
  pvPower = pvVoltage * pvCurrent;
}

// ========================================================================
// DATA LOGGING FUNCTION
// ========================================================================

/*
 * Function: sendDataToExcel
 * -------------------------
 * Sends real-time data to Excel spreadsheet using PLX-DAQ protocol. 
 * Data includes:
 * - Timestamp (automatically generated by PLX-DAQ)
 * - Current mode (Automatic/Manual)
 * - PV voltage, current, and power
 * - Current servo angles (horizontal and vertical)
 * 
 * FORMAT:  DATA,Time,Mode,Voltage,Current,Power,HorizontalAngle,VerticalAngle
 */
void sendDataToExcel() {
  // Start data row with PLX-DAQ command
  Serial.print("DATA,");
  
  // Timestamp (automatically handled by PLX-DAQ)
  Serial.print("TIME,");
  
  // Current mode
  if (isAutoMode) {
    Serial.print("Automatic,");
  } else {
    Serial.print("Manual,");
  }
  
  // PV panel voltage (in volts)
  Serial.print(pvVoltage, 3);  // Print with 3 decimal places
  Serial.print(",");
  
  // PV panel current (in amperes)
  Serial.print(pvCurrent, 3);  // Print with 3 decimal places
  Serial.print(",");
  
  // PV panel power (in watts)
  Serial.print(pvPower, 3);  // Print with 3 decimal places
  Serial.print(",");
  
  // Horizontal servo angle (azimuth position)
  Serial.print(currentHorizontalAngle);
  Serial.print(",");
  
  // Vertical servo angle (elevation position)
  Serial.println(currentVerticalAngle);  // println adds newline character
}

/*
 * ========================================================================
 * END OF CODE
 * ========================================================================
 * 
 * USAGE INSTRUCTIONS:
 * 
 * 1. HARDWARE SETUP:
 *    - Connect all components according to pin definitions above
 *    - Ensure proper power supply for servo motors (external 5V recommended)
 *    - Connect Arduino to computer via USB cable
 * 
 * 2. SOFTWARE SETUP:
 *    - Upload this code to Arduino Uno R3 using Arduino IDE
 *    - Download and install PLX-DAQ Excel Macro
 *    - Open PLX-DAQ Spreadsheet from installation folder
 * 
 * 3. OPERATION:
 *    - In PLX-DAQ window, select correct COM port and baud rate (9600)
 *    - Click "Connect" button to start data logging
 *    - Use mode switch button to toggle between automatic and manual modes
 *    - In manual mode, use motor select button to choose which servo to control
 *    - In manual mode, turn potentiometer to adjust servo position
 *    - Watch real-time data appear in Excel spreadsheet
 * 
 * 4. CALIBRATION:
 *    - Adjust LIGHT_TOLERANCE if tracker is too sensitive/insensitive
 *    - Adjust NIGHT_THRESHOLD based on your LDR sensors
 *    - Modify SUNRISE_HORIZONTAL and SUNRISE_VERTICAL for your location
 *    - Update VOLTAGE_DIVIDER and LOAD_RESISTOR for accurate measurements
 * 
 * TROUBLESHOOTING:
 * - If servos jitter:  Increase LIGHT_TOLERANCE value
 * - If no Excel data: Check COM port and baud rate settings
 * - If servos don't move: Check power supply and connections
 * - If inaccurate voltage: Calibrate VOLTAGE_DIVIDER constant
 * 
 * ========================================================================
 */