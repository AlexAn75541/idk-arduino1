/*
 * Solar Tracker - Group 4 SEMI
 * Bộ Theo Dõi Mặt Trời - Nhóm 4 SEMI
 * Hardware: Arduino Uno R3, 2 servos, 4 LDRs
 * Phần cứng: Arduino Uno R3, 2 servo, 4 cảm biến ánh sáng
 * Modes: Auto tracking, Manual control, Excel logging
 * Chế độ: Tự động theo dõi, Điều khiển thủ công, Ghi dữ liệu Excel(đã được lược bỏ)
 */

#include <Servo.h>

// --- Pin Definitions ---
// --- Định nghĩa chân ---
#define LDR_TL_PIN A0          // Top-Left LDR / Cảm biến trên-trái
#define LDR_TR_PIN A1          // Top-Right LDR / Cảm biến trên-phải
#define LDR_BL_PIN A2          // Bottom-Left LDR / Cảm biến dưới-trái
#define LDR_BR_PIN A3          // Bottom-Right LDR / Cảm biến dưới-phải

#define POT_PIN A4             // Potentiometer for manual control / Biến trở điều khiển thủ công
#define VOLT_PIN A5            // Solar panel voltage sensor / Cảm biến điện áp pin mặt trời

#define SERVO_LR_PIN 6         // Left-Right servo (continuous) / Servo trái-phải (quay liên tục)
#define SERVO_UD_PIN 5         // Up-Down servo (180°) / Servo lên-xuống (180°)

#define BTN_MODE_PIN 12        // Mode switch button (Auto/Manual) / Nút chuyển chế độ (Tự động/Thủ công)
#define BTN_AXIS_PIN 11        // Axis switch button (LR/UD in Manual) / Nút chuyển trục (LR/UD khi Thủ công)

// --- Constants ---
// --- Hằng số ---
#define TOLERANCE 10           // Light difference threshold for servo movement / Ngưỡng chênh lệch ánh sáng để servo di chuyển
#define NIGHT_LIMIT 8          // Night mode threshold (low light) / Ngưỡng chế độ đêm (ánh sáng yếu)
#define LOAD_RESISTANCE 10.0   // Load resistance (10Ω) for current calc / Điện trở tải (10Ω) để tính dòng điện

// Continuous Servo Speeds (90 = stop)
// Tốc độ Servo liên tục (90 = dừng)
#define LR_SPEED_RIGHT 80      // Rotate right (<90) / Quay phải (<90)
#define LR_SPEED_LEFT 100      // Rotate left (>90) / Quay trái (>90)
#define LR_STOP 90             // Stop position / Vị trí dừng

Servo servoLR;                 // Left-Right servo object / Đối tượng servo trái-phải
Servo servoUD;                 // Up-Down servo object / Đối tượng servo lên-xuống

// --- Variables ---
// --- Biến ---
bool isAutoMode = true;        // Current mode (true=Auto, false=Manual) / Chế độ hiện tại (true=Tự động, false=Thủ công)
bool manualControlLR = true;   // Manual axis (true=LR, false=UD) / Trục thủ công (true=LR, false=UD)

int posUD = 45;                // Up-Down servo position / Vị trí servo lên-xuống

// Button states for debouncing
// Trạng thái nút để chống rung
int lastBtnModeState = HIGH;   // Previous mode button state / Trạng thái nút chế độ trước
int lastBtnAxisState = HIGH;   // Previous axis button state / Trạng thái nút trục trước

void setup() {
  Serial.begin(9600);                  // Start serial at 9600 baud / Bắt đầu serial ở tốc độ 9600 baud
  Serial.println("CLEARDATA");         // Clear Excel data (PLX-DAQ) / Xóa dữ liệu Excel (PLX-DAQ)
  Serial.println("LABEL,Time,Mode,Voltage(V),Current(mA),Power(mW)");  // Excel headers / Tiêu đề Excel

  servoLR.attach(SERVO_LR_PIN);        // Attach LR servo / Gắn servo LR
  servoUD.attach(SERVO_UD_PIN);        // Attach UD servo / Gắn servo UD
  
  pinMode(BTN_MODE_PIN, INPUT_PULLUP); // Mode button with pullup / Nút chế độ với pullup
  pinMode(BTN_AXIS_PIN, INPUT_PULLUP); // Axis button with pullup / Nút trục với pullup
  
  // Initialize servo positions / Khởi tạo vị trí servo
  servoLR.write(LR_STOP);              // Stop LR servo / Dừng servo LR
  servoUD.write(posUD);                // Set UD to 45° / Đặt UD ở 45°
  
  delay(500);                          // Allow servos to reach position / Cho servo về vị trí
}

void loop() {
  // --- 1. Check Mode Button ---
  // --- 1. Kiểm tra nút chế độ ---
  int btnModeState = digitalRead(BTN_MODE_PIN);
  if (btnModeState == LOW && lastBtnModeState == HIGH) {  // Button pressed / Nút được nhấn
    isAutoMode = !isAutoMode;          // Toggle mode / Chuyển đổi chế độ
    servoLR.write(LR_STOP);            // Stop servo / Dừng servo
    delay(200);                        // Debounce / Chống rung
  }
  lastBtnModeState = btnModeState;

  // --- 2. Run Current Mode ---
  // --- 2. Chạy chế độ hiện tại ---
  if (isAutoMode) {
    runAutomaticMode();                // Auto tracking / Theo dõi tự động
  } else {
    runManualMode();                   // Manual control / Điều khiển thủ công
  }

  // --- 3. Log Data ---
  // --- 3. Ghi dữ liệu ---
  logDataToExcel();                    // Send data to Excel / Gửi dữ liệu đến Excel
  
  delay(50);                           // Loop delay / Trễ vòng lặp
}

// --- Automatic Mode ---
// --- Chế độ tự động ---
void runAutomaticMode() {
  // 1. Read LDR sensors / Đọc cảm biến LDR
  int topleft = analogRead(LDR_TL_PIN);
  int topright = analogRead(LDR_TR_PIN);
  int botleft = analogRead(LDR_BL_PIN);
  int botright = analogRead(LDR_BR_PIN);

  // 2. Calculate averages / Tính giá trị trung bình
  int avgtop = (topright + topleft) / 2;
  int avgbot = (botright + botleft) / 2;
  int avgright = (topright + botright) / 2;
  int avgleft = (topleft + botleft) / 2;

  // 3. Calculate differences / Tính độ chênh lệch
  int diffelev = avgtop - avgbot;          // Elevation diff (+top brighter) / Chênh cao độ (+trên sáng hơn)
  int diffazi = avgright - avgleft;        // Azimuth diff (+right brighter) / Chênh phương vị (+phải sáng hơn)
  int avgsum = (avgtop + avgbot + avgright + avgleft) / 4;

  // 4. Night mode check / Kiểm tra chế độ đêm
  if (avgsum < NIGHT_LIMIT) {              // Too dark / Quá tối
    servoLR.write(LR_STOP);                // Stop LR / Dừng LR
    posUD = 30;                            // Face east for sunrise / Hướng đông đón bình minh
    servoUD.write(posUD);
    return;
  }

  // 5. Azimuth (Left-Right) control / Điều khiển phương vị (Trái-Phải)
  if (abs(diffazi) <= TOLERANCE) {         // Within tolerance / Trong ngưỡng
    servoLR.write(LR_STOP);                // Stop (aligned) / Dừng (đã căn chỉnh)
  } 
  else {                                   // Need adjustment / Cần điều chỉnh
    if (diffazi > 0) {                     // Brighter on right / Sáng hơn bên phải
      servoLR.write(LR_SPEED_RIGHT);       // Move right / Di chuyển sang phải
    } else {                               // Brighter on left / Sáng hơn bên trái
      servoLR.write(LR_SPEED_LEFT);        // Move left / Di chuyển sang trái
    }
  }

  // 6. Elevation (Up-Down) control / Điều khiển cao độ (Lên-Xuống)
  if (abs(diffelev) <= TOLERANCE) {        // Within tolerance / Trong ngưỡng
    // No change (aligned) / Không thay đổi (đã căn chỉnh)
  } 
  else {                                   // Need adjustment / Cần điều chỉnh
    if (diffelev > 0) {                    // Brighter on top / Sáng hơn ở trên
      posUD = posUD + 1;                   // Tilt up / Nghiêng lên
    } else {                               // Brighter on bottom / Sáng hơn ở dưới
      posUD = posUD - 1;                   // Tilt down / Nghiêng xuống
    }
    posUD = constrain(posUD, 0, 180);      // Limit to 0-180° / Giới hạn 0-180°
    servoUD.write(posUD);
  }
}

// --- Manual Mode ---
// --- Chế độ thủ công ---
void runManualMode() {
  int btnAxisState = digitalRead(BTN_AXIS_PIN);
  if (btnAxisState == LOW && lastBtnAxisState == HIGH) {  // Button pressed / Nút được nhấn
    manualControlLR = !manualControlLR;      // Toggle axis / Chuyển trục
    servoLR.write(LR_STOP);                  // Stop servo / Dừng servo
    delay(200);                              // Debounce / Chống rung
  }
  lastBtnAxisState = btnAxisState;

  int potVal = analogRead(POT_PIN);          // Read potentiometer / Đọc biến trở
  int angle = map(potVal, 0, 1023, 0, 180);  // Map to 0-180° / Ánh xạ 0-180°

  if (manualControlLR) {                     // Control LR servo / Điều khiển servo LR
    servoLR.write(angle);                    // Set speed/direction / Đặt tốc độ/hướng
  } else {                                   // Control UD servo / Điều khiển servo UD
    servoUD.write(angle);                    // Set position / Đặt vị trí
    posUD = angle;
  }
}


// PHẦN ĐÃ BỊ LƯỢC BỎ


// --- Data Logging ---
// --- Ghi dữ liệu ---
void logDataToExcel() {
  float sensorValue = analogRead(VOLT_PIN);
  float voltage = sensorValue * (5.0 / 1023.0);  // Convert to volts / Chuyển đổi sang volt
  float current = (voltage / LOAD_RESISTANCE) * 1000;  // Current (mA) / Dòng điện (mA)
  float power = voltage * current;             // Power (mW) / Công suất (mW)

  Serial.print("DATA,TIME,");                  // PLX-DAQ format / Định dạng PLX-DAQ
  if (isAutoMode) Serial.print("Auto");
  else Serial.print("Manual");
  Serial.print(",");
  Serial.print(voltage);
  Serial.print(",");
  Serial.print(current);
  Serial.print(",");
  Serial.println(power);
}
