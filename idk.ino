#include <Servo.h>

#define LDR_TL_PIN A0
#define LDR_TR_PIN A1
#define LDR_BL_PIN A2
#define LDR_BR_PIN A3

#define BUT1_pin 4
#define BUT1_pin 5

#define SERVO_AZIMUTH 9
#define SERVO_ALTITUDE 10

#define LDR_DEADZONE 40

#define SERVO_CONTROL_INTERVAL_MS 500

int ldr_tl_val;
int ldr_tr_val;
int ldr_bl_val;
int ldr_br_val;

long prev_time;

Servo servo_azi;
Servo servo_alt;

int servo_azi_val;
int servo_alt_val;

void setup()
{

  Serial.begin(9600);

  servo_azi.attach(9);
  servo_alt.attach(10);
}

void loop()
{

  unsigned long time = millis();

  ldr_tl_val = analogRead(A0);
  ldr_tr_val = analogRead(A1);
  ldr_bl_val = analogRead(A2);
  ldr_br_val = analogRead(A3);

  int ldr_avg_t = (ldr_tl_val + ldr_tr_val) / 2;
  int ldr_avg_b = (ldr_bl_val + ldr_br_val) / 2;
  int ldr_avg_r = (ldr_tr_val + ldr_br_val) / 2;
  int ldr_avg_l = (ldr_tl_val + ldr_bl_val) / 2;

  int ldr_avg = (ldr_tl_val + ldr_tr_val + ldr_bl_val + ldr_br_val) / 4;

  int diff_azi = ldr_avg_r - ldr_avg_l;
  int diff_alt = ldr_avg_t - ldr_avg_b;

  if (abs(diff_azi) > LDR_DEADZONE)
  {
    if (diff_azi < 0)
    {
      servo_azi_val -= 1; servo_azi_val = constrain(servo_azi_val, 0, 180);
    }
    else
    {
      servo_azi_val += 1;
      servo_azi_val = constrain(servo_azi_val, 0, 180);
    }
  }

  if (abs(diff_alt) > LDR_DEADZONE)
  {
    if (diff_alt < 0)
    {
      servo_alt_val -= 1; servo_alt_val = constrain(servo_alt_val, 0, 180);
    }
    else
    {
      servo_alt_val += 1;
      servo_alt_val = constrain(servo_alt_val, 0, 180);
    }
  }

  servo_azi.write(servo_azi_val);
  servo_alt.write(servo_alt_val);

}