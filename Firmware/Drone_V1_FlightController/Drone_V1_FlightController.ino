#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>

/*
============================================
 PINOUT REFERENCE - Arduino Nano
============================================
ESC OUTPUTS (motor control, PWM via Servo.h):
D3  -> ESC1 (Front-Right)
D4  -> ESC2 (Front-Left)
D5  -> ESC3 (Rear-Right)
D6  -> ESC4 (Rear-Left)

RECEIVER INPUTS (FlySky i6, stock PWM receiver):
D9  -> Roll channel
D10 -> Pitch channel
D11 -> Throttle channel
D12 -> Yaw channel

MPU6050 (IMU, I2C):
A4  -> SDA
A5  -> SCL

MISC:
D13 (LED_BUILTIN) -> heartbeat / status blink

POWER:
5V rail from single ESC BEC, all grounds common
============================================
*/

// ==== SET TO true ONLY WHEN YOU WANT TO RUN ESC CALIBRATION ====
// PROPS OFF. Battery must be DISCONNECTED before flashing with this true.
// After calibrating, set back to false and reflash before ever flying.
bool CALIBRATE_ESCS = false;

// ESC pins
const int ESC1_PIN = 3;  // Front-Right
const int ESC2_PIN = 4;  // Front-Left
const int ESC3_PIN = 5;  // Rear-Right
const int ESC4_PIN = 6;  // Rear-Left

// Receiver pins
const int CH_ROLL_PIN = 9;
const int CH_PITCH_PIN = 10;
const int CH_THROTTLE_PIN = 11;
const int CH_YAW_PIN = 12;

Servo esc1, esc2, esc3, esc4;

// Input of channels
int roll_in = 0;
int pitch_in = 0;
int throt_in = 0;
int yaw_in = 0;

int THROTTLE_LOW_THRESH = 1180;

// For LED blinking & printing (separate timers, no conflict)
unsigned long timer1 = 0;
unsigned long timer2 = 0;
bool LED_state = LOW;

// MPU init & angle/rate readings
MPU6050 mpu(Wire);
float roll_angle = 0;
float pitch_angle = 0;
float yaw_rate = 0;

// For calculating dt
unsigned long last_time;
float dt;

// PID working variables
float roll_setpoint, pitch_setpoint, yaw_rate_setpoint;
float roll_prop, pitch_prop, yaw_prop;
float roll_integ, pitch_integ, yaw_integ;
float roll_derv, pitch_derv, yaw_derv;
float roll_pid, pitch_pid, yaw_pid;

// PID gains - starting point from MATLAB tuning using real CAD inertia
float roll_kp = 1.2,  roll_ki = 0.05,  roll_kd = 0.27;
float pitch_kp = 1.2, pitch_ki = 0.05, pitch_kd = 0.27;
float yaw_kp = 1.2,   yaw_ki = 0.15,   yaw_kd = 0.0;

float prev_roll_error, prev_pitch_error, prev_yaw_error;

// Final motor output values (global, single source of truth - no shadowing)
int FR = 0;
int FL = 0;
int RR = 0;
int RL = 0;

// ---- Optional throttle softening curve ----
bool USE_THROTTLE_CURVE = true;
float THROTTLE_CURVE_EXP = 1.4; // >1 = softer low end. Try 1.2-1.6.

int applyThrottleCurve(int raw_throttle) {
  if (!USE_THROTTLE_CURVE) return raw_throttle;
  float pct = (raw_throttle - 1000) / 1000.0;
  pct = constrain(pct, 0.0, 1.0);
  float curved = pow(pct, THROTTLE_CURVE_EXP);
  return 1000 + (int)(curved * 1000);
}


void print_values() {
  if ((millis() - timer1) > 100) {
    Serial.print("X : "); Serial.print(mpu.getAngleX());
    Serial.print("\tY : "); Serial.print(mpu.getAngleY());
    Serial.print("\tZ : "); Serial.println(mpu.getAngleZ());
    Serial.println();

    Serial.print("\tRoll : "); Serial.println(roll_in);
    Serial.print("\tPitch : "); Serial.println(pitch_in);
    Serial.print("\tThrot : "); Serial.println(throt_in);
    Serial.print("\tYaw : "); Serial.println(yaw_in);
    Serial.println();

    Serial.print("\tFR : "); Serial.println(FR);
    Serial.print("\tFL : "); Serial.println(FL);
    Serial.print("\tRR : "); Serial.println(RR);
    Serial.print("\tRL : "); Serial.println(RL);
    Serial.println();

    timer1 = millis();
  }
}

void LED_blink(int time) {
  if (millis() - timer2 >= time) {
    timer2 = millis();
    LED_state = !LED_state;
    digitalWrite(LED_BUILTIN, LED_state);
  }
}

int readChannel(int pin, int safe_val) {
  int val = pulseIn(pin, HIGH, 25000);
  if (val == 0) {
    val = safe_val;
  }
  val = constrain(val, 1000, 2000);
  return val;
}


void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(CH_ROLL_PIN, INPUT);
  pinMode(CH_PITCH_PIN, INPUT);
  pinMode(CH_THROTTLE_PIN, INPUT);
  pinMode(CH_YAW_PIN, INPUT);

  pinMode(ESC1_PIN, OUTPUT);
  pinMode(ESC2_PIN, OUTPUT);
  pinMode(ESC3_PIN, OUTPUT);
  pinMode(ESC4_PIN, OUTPUT);

  Serial.begin(250000);

  esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);
  esc3.attach(ESC3_PIN, 1000, 2000);
  esc4.attach(ESC4_PIN, 1000, 2000);

  // ==== ESC CALIBRATION BLOCK ====
  if (CALIBRATE_ESCS) {
    esc1.writeMicroseconds(2000);
    esc2.writeMicroseconds(2000);
    esc3.writeMicroseconds(2000);
    esc4.writeMicroseconds(2000);

    delay(5000);

    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);

    delay(5000);

    while (true) {
      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
      esc3.writeMicroseconds(1000);
      esc4.writeMicroseconds(1000);
    }
  }
  // ==== END CALIBRATION BLOCK ====

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(3000);

  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { LED_blink(100); }

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Done!\n");

  throt_in = readChannel(CH_THROTTLE_PIN, 1000);
  while (throt_in >= THROTTLE_LOW_THRESH) {
    throt_in = readChannel(CH_THROTTLE_PIN, 1000);
    LED_blink(1000);
  }

  last_time = micros();
}


void loop() {
  roll_in = readChannel(CH_ROLL_PIN, 1500);
  pitch_in = readChannel(CH_PITCH_PIN, 1500);
  throt_in = readChannel(CH_THROTTLE_PIN, 1000);
  yaw_in = readChannel(CH_YAW_PIN, 1500);

  int throt_curved = applyThrottleCurve(throt_in);

  unsigned long now = micros();
  dt = (now - last_time) / 1000000.0;
  if (dt <= 0) {
    dt = 0.00001;
  }
  last_time = now;

  roll_setpoint = map(roll_in, 1000, 2000, -15, 15);
  pitch_setpoint = map(pitch_in, 1000, 2000, -15, 15);
  yaw_rate_setpoint = map(yaw_in, 1000, 2000, -180, 180);

  mpu.update();
  roll_angle = - mpu.getAngleX();
  pitch_angle = - mpu.getAngleY();
  yaw_rate = - mpu.getGyroZ();

  float roll_error = roll_setpoint - roll_angle;
  float pitch_error = pitch_setpoint - pitch_angle;
  float yaw_error = yaw_rate_setpoint - yaw_rate;

  if (throt_in < THROTTLE_LOW_THRESH) {
    roll_integ = 0;
    pitch_integ = 0;
    yaw_integ = 0;

    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);

  } else {
    roll_prop = roll_kp * roll_error;
    pitch_prop = pitch_kp * pitch_error;
    yaw_prop = yaw_kp * yaw_error;

    roll_integ  += roll_ki * roll_error * dt;
    pitch_integ += pitch_ki * pitch_error * dt;
    yaw_integ   += yaw_ki * yaw_error * dt;

    roll_derv = roll_kd * (roll_error - prev_roll_error) / dt;
    pitch_derv = pitch_kd * (pitch_error - prev_pitch_error) / dt;
    yaw_derv = yaw_kd * (yaw_error - prev_yaw_error) / dt;

    prev_roll_error = roll_error;
    prev_pitch_error = pitch_error;
    prev_yaw_error = yaw_error;

    roll_pid = roll_prop + roll_integ + roll_derv;
    pitch_pid = pitch_prop + pitch_integ + pitch_derv;
    yaw_pid = yaw_prop + yaw_integ + yaw_derv;

    roll_pid  = constrain(roll_pid,  -400, 400);
    pitch_pid = constrain(pitch_pid, -400, 400);
    yaw_pid   = constrain(yaw_pid,   -400, 400);

    FR = throt_curved - roll_pid + pitch_pid - yaw_pid;
    FL = throt_curved + roll_pid + pitch_pid + yaw_pid;
    RR = throt_curved - roll_pid - pitch_pid + yaw_pid;
    RL = throt_curved + roll_pid - pitch_pid - yaw_pid;

    FR = constrain(FR, 1000, 2000);
    FL = constrain(FL, 1000, 2000);
    RR = constrain(RR, 1000, 2000);
    RL = constrain(RL, 1000, 2000);

    esc1.writeMicroseconds(FR);
    esc2.writeMicroseconds(FL);
    esc3.writeMicroseconds(RR);
    esc4.writeMicroseconds(RL);

    print_values();
  }
}