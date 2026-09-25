#include <Wire.h> 
#include <MPU6050_light.h> 
#include <avr/interrupt.h> 
 
 
/* 
 
 PINOUT REFERENCE - Arduino Nano 
------------------------------------ 
ESC OUTPUTS: 
D3  -> ESC1 (Front-Right) 
D4  -> ESC2 (Front-Left) 
D5  -> ESC3 (Rear-Right) 
D6  -> ESC4 (Rear-Left) 
 
RECEIVER INPUTS: 
D9  -> Roll channel 
D10 -> Pitch channel 
D11 -> Throttle channel 
D12 -> Yaw channel 
 
MPU6050 (IMU, I2C): 
A4  -> SDA 
A5  -> SCL 
 
*/ 
 
 
// ----Debug----  
bool DEBUG_OUTPUT = false;  
unsigned long debug_timer = 0; 
 
 
 
// ----ESC---- 
const int ESC1_PIN = 3;  // Front-Right 
const int ESC2_PIN = 4;  // Front-Left 
const int ESC3_PIN = 5;  // Rear-Right 
const int ESC4_PIN = 6;  // Rear-Left 
 
// ----Receiver pins---- 
const int CH_ROLL_PIN = 9; 
const int CH_PITCH_PIN = 10; 
const int CH_THROTTLE_PIN = 11; 
const int CH_YAW_PIN = 12; 
 
//----Inputs ----  
int roll_in = 1500;  
int pitch_in = 1500;  
int throttle_in = 1000;  
int yaw_in = 1500; 


// ----Receiver Calibration----
// Assumed receiver values: LOW = 1000, CENTER = 1500, HIGH = 2000
const int ROLL_LOW = 1000;
const int ROLL_CENTER = 1500;
const int ROLL_HIGH = 2000;

const int PITCH_LOW = 1000;
const int PITCH_CENTER = 1500;
const int PITCH_HIGH = 2000;

const int THROTTLE_LOW = 1000;
const int THROTTLE_CENTER = 1500;
const int THROTTLE_HIGH = 2000;

const int YAW_LOW = 1000;
const int YAW_CENTER = 1500;
const int YAW_HIGH = 2000;


// ----Receiver Interrupt Variables---- 
volatile int receiver_roll = 1500; 
volatile int receiver_pitch = 1500; 
volatile int receiver_throttle = 1000; 
volatile int receiver_yaw = 1500; 

volatile unsigned long roll_timer = 0; 
volatile unsigned long pitch_timer = 0; 
volatile unsigned long throttle_timer = 0; 
volatile unsigned long yaw_timer = 0; 

volatile unsigned long roll_last_update = 0; 
volatile unsigned long pitch_last_update = 0; 
volatile unsigned long throttle_last_update = 0; 
volatile unsigned long yaw_last_update = 0; 

volatile bool roll_high = false; 
volatile bool pitch_high = false; 
volatile bool throttle_high = false; 
volatile bool yaw_high = false; 

bool receiver_ok = false;


// ----Objects---- 
MPU6050 mpu(Wire); 
 
// ----IMU----  
float roll_angle  = 0; 
float pitch_angle = 0; 
 
float gyro_roll  = 0; 
float gyro_pitch = 0; 
float gyro_yaw   = 0; 
 
// Filtered gyro rates 
float gyro_roll_input  = 0; 
float gyro_pitch_input = 0; 
float gyro_yaw_input   = 0; 
 
//----Auto-Level---- 
bool AUTO_LEVEL = true; 
float roll_level_adjust  = 0; 
float pitch_level_adjust = 0; 
 
 
// ----PID---- 
float roll_kp = 1.3; 
float roll_ki = 10.0; 
float roll_kd = 0.072; 
 
float pitch_kp = 1.3; 
float pitch_ki = 10.0; 
float pitch_kd = 0.072; 
 
float yaw_kp = 4.0; 
float yaw_ki = 5.0; 
float yaw_kd = 0.0; 
 
const float MAX_PID = 400; 
 
// Setpoints are angular rates 
float roll_rate_setpoint  = 0; 
float pitch_rate_setpoint = 0; 
float yaw_rate_setpoint   = 0; 
 
// Integrals 
float roll_integral  = 0; 
float pitch_integral = 0; 
float yaw_integral   = 0; 
 
// Previous errors 
float prev_roll_error  = 0; 
float prev_pitch_error = 0; 
float prev_yaw_error   = 0; 
 
// PID outputs 
float roll_pid  = 0; 
float pitch_pid = 0; 
float yaw_pid   = 0; 

bool just_armed = false;
 
 
// ----Motor Outputs---- 
int FR = 1000; 
int FL = 1000; 
int RR = 1000; 
int RL = 1000; 
 
volatile int esc_FR = 1000; 
volatile int esc_FL = 1000; 
volatile int esc_RR = 1000; 
volatile int esc_RL = 1000; 
 
 
// ----Timer---- 
unsigned long last_time = 0; 
const unsigned long CONTROL_LOOP_US = 4000; 
float dt = 0.004; // Fixed 4 ms (250 Hz) control loop
 
 
 
// ----Arming Drone---- 
bool armed = false; 
const unsigned long ARM_HOLD_MS = 2000; 
unsigned long arm_hold_start = 0; 
unsigned long disarm_hold_start = 0; 
 
 
void droneArm() 
{ 
  unsigned long now = millis(); 
  
  if (!armed) 
  { 
    if (throttle_in < 1080 && yaw_in > 1900 ) 
    { 
      if (arm_hold_start == 0) 
      { 
        arm_hold_start = now;  
      } 
 
      if(now - arm_hold_start > ARM_HOLD_MS) 
      { 
        armed = true;  
        just_armed = true;
        arm_hold_start = 0;  
 
        // Reset integrals 
        roll_integral = 0; 
        pitch_integral = 0; 
        yaw_integral = 0; 
      } 
    } 
 
    else 
    { 
      arm_hold_start = 0;  
    } 
  } 
 
  if (armed) 
  { 
    if (throttle_in < 1080 && yaw_in < 1100 ) 
    { 
      if (disarm_hold_start == 0) 
      { 
        disarm_hold_start = now;  
      } 
 
      if(now - disarm_hold_start > ARM_HOLD_MS) 
      { 
        armed = false;  
        disarm_hold_start = 0;  
      } 
    } 
 
    else 
    { 
      disarm_hold_start = 0;  
    } 
  } 
} 
 
 
 
void updateESCCommands() { 
 
  noInterrupts(); 
 
  esc_FR = FR; 
  esc_FL = FL; 
  esc_RR = RR; 
  esc_RL = RL; 
 
  interrupts(); 
} 


void setupESCTimer() { 

  noInterrupts(); 

  TCCR1A = 0; 
  TCCR1B = 0; 
  TCNT1 = 0; 

  // Timer1 clock = 16 MHz / 8 = 2 MHz 
  // Each timer tick = 0.5 microseconds 
  TCCR1B |= (1 << CS11); 

  OCR1A = 100; 

  TIMSK1 |= (1 << OCIE1A); 

  interrupts(); 
} 


void setupReceiverInterrupts() { 

  // Enable pin-change interrupts for PORTB 
  PCICR |= (1 << PCIE0); 

  // D9  = PCINT1 
  // D10 = PCINT2 
  // D11 = PCINT3 
  // D12 = PCINT4 
  PCMSK0 |= (1 << PCINT1); 
  PCMSK0 |= (1 << PCINT2); 
  PCMSK0 |= (1 << PCINT3); 
  PCMSK0 |= (1 << PCINT4); 

  PCIFR |= (1 << PCIF0); 
} 


int calibrateReceiverChannel(int value, int low, int center, int high) {

  value = constrain(value, low, high);

  if (value < center) 
  {
    return map(value, low, center, 1000, 1500);
  }

  if (value > center) 
  {
    return map(value, center, high, 1500, 2000);
  }

  return 1500;
}


void readReceiver() { 

  int temp_roll; 
  int temp_pitch; 
  int temp_throttle; 
  int temp_yaw; 

  unsigned long temp_roll_update; 
  unsigned long temp_pitch_update; 
  unsigned long temp_throttle_update; 
  unsigned long temp_yaw_update; 

  noInterrupts(); 

  temp_roll = receiver_roll; 
  temp_pitch = receiver_pitch; 
  temp_throttle = receiver_throttle; 
  temp_yaw = receiver_yaw; 

  temp_roll_update = roll_last_update; 
  temp_pitch_update = pitch_last_update; 
  temp_throttle_update = throttle_last_update; 
  temp_yaw_update = yaw_last_update; 

  interrupts(); 

  unsigned long now = micros(); 

  // Receiver failsafe 
  receiver_ok =
    (now - temp_roll_update <= 100000) &&
    (now - temp_pitch_update <= 100000) &&
    (now - temp_throttle_update <= 100000) &&
    (now - temp_yaw_update <= 100000);

  if (!receiver_ok) 
  {
    roll_in = 1500;
    pitch_in = 1500;
    throttle_in = 1000;
    yaw_in = 1500;

    armed = false;

    return;
  }

  roll_in = calibrateReceiverChannel(
    temp_roll, ROLL_LOW, ROLL_CENTER, ROLL_HIGH
  );

  pitch_in = calibrateReceiverChannel(
    temp_pitch, PITCH_LOW, PITCH_CENTER, PITCH_HIGH
  );

  throttle_in = calibrateReceiverChannel(
    temp_throttle, THROTTLE_LOW, THROTTLE_CENTER, THROTTLE_HIGH
  );

  yaw_in = calibrateReceiverChannel(
    temp_yaw, YAW_LOW, YAW_CENTER, YAW_HIGH
  );
} 
 
 
void debugOutput() { 
 
  if (!DEBUG_OUTPUT) 
  { 
    return;  
  } 
 
  if (millis() - debug_timer < 100) { 
    return; 
  } 
 
  debug_timer = millis();  
 
  Serial.print("ARM: "); 
  Serial.print(armed); 

  Serial.print(" RC: ");
  Serial.print(receiver_ok);
 
  Serial.print(" | ANG R: "); 
  Serial.print(roll_angle); 
 
  Serial.print(" P: "); 
  Serial.print(pitch_angle); 
 
  Serial.print(" | GYRO R: "); 
  Serial.print(gyro_roll_input); 
  Serial.print(" P: "); 
  Serial.print(gyro_pitch_input); 
  Serial.print(" Y: "); 
  Serial.print(gyro_yaw_input); 
 
  Serial.print(" | PID R: "); 
  Serial.print(roll_pid); 
  Serial.print(" P: "); 
  Serial.print(pitch_pid); 
  Serial.print(" Y: "); 
  Serial.print(yaw_pid); 
 
  Serial.print(" | FR: "); 
  Serial.print(FR); 
  Serial.print(" FL: "); 
  Serial.print(FL); 
  Serial.print(" RR: "); 
  Serial.print(RR); 
  Serial.print(" RL: "); 
  Serial.println(RL); 
} 
 
 
 
void calculateSetpoints() { 

  // Auto-level correction 
  roll_level_adjust = roll_angle * 15.0; 
  pitch_level_adjust = pitch_angle * 15.0; 
 
  if (!AUTO_LEVEL) { 
    roll_level_adjust = 0; 
    pitch_level_adjust = 0; 
  } 
 
  // Roll 
  roll_rate_setpoint = 0; 
 
  if (roll_in > 1508) { 
    roll_rate_setpoint = roll_in - 1508; 
  } else if (roll_in < 1492) { 
    roll_rate_setpoint = roll_in - 1492; 
  } 
 
  roll_rate_setpoint -= roll_level_adjust; 
  roll_rate_setpoint /= 3.0; 
 
  // Pitch 
  pitch_rate_setpoint = 0; 
 
  if (pitch_in > 1508) { 
    pitch_rate_setpoint = pitch_in - 1508; 
  } else if (pitch_in < 1492) { 
    pitch_rate_setpoint = pitch_in - 1492; 
  } 
 
  pitch_rate_setpoint -= pitch_level_adjust; 
  pitch_rate_setpoint /= 3.0; 
 
  // Yaw 
  yaw_rate_setpoint = 0; 
 
  if (throttle_in > 1050) { 
    if (yaw_in > 1508) { 
      yaw_rate_setpoint = (yaw_in - 1508) / 3.0; 
    } else if (yaw_in < 1492) { 
      yaw_rate_setpoint = (yaw_in - 1492) / 3.0; 
    } 
  } 
} 
 
 
void calculatePID() { 

  // Roll 
  float roll_error = gyro_roll_input - roll_rate_setpoint; 

  if (just_armed)
  {
    prev_roll_error = roll_error;
  }

  roll_integral += roll_ki * roll_error * dt; 
  roll_integral = constrain(roll_integral, -MAX_PID, MAX_PID); 
   
  float roll_derivative = (roll_error - prev_roll_error) / dt; 
  roll_pid = roll_kp * roll_error + roll_integral + roll_kd * roll_derivative; 
  roll_pid = constrain(roll_pid, -MAX_PID, MAX_PID); 
  prev_roll_error = roll_error; 
 
 
  // Pitch 
  float pitch_error = gyro_pitch_input - pitch_rate_setpoint; 

  if (just_armed)
  {
    prev_pitch_error = pitch_error;
  }

  pitch_integral += pitch_ki * pitch_error * dt; 
  pitch_integral = constrain(pitch_integral, -MAX_PID, MAX_PID); 
   
  float pitch_derivative = (pitch_error - prev_pitch_error) / dt; 
  pitch_pid = pitch_kp * pitch_error + pitch_integral + pitch_kd * pitch_derivative; 
  pitch_pid = constrain(pitch_pid, -MAX_PID, MAX_PID); 
  prev_pitch_error = pitch_error; 
 
 
  // Yaw 
  float yaw_error = gyro_yaw_input - yaw_rate_setpoint; 

  if (just_armed)
  {
    prev_yaw_error = yaw_error;
  }

  yaw_integral += yaw_ki * yaw_error * dt; 
  yaw_integral = constrain(yaw_integral, -MAX_PID, MAX_PID); 
   
  float yaw_derivative = (yaw_error - prev_yaw_error) / dt; 
  yaw_pid = yaw_kp * yaw_error + yaw_integral + yaw_kd * yaw_derivative; 
  yaw_pid = constrain(yaw_pid, -MAX_PID, MAX_PID); 
  prev_yaw_error = yaw_error; 

  just_armed = false;
} 


void resetPID()
{
  roll_integral = 0;
  pitch_integral = 0;
  yaw_integral = 0;

  prev_roll_error = 0;
  prev_pitch_error = 0;
  prev_yaw_error = 0;

  roll_pid = 0;
  pitch_pid = 0;
  yaw_pid = 0;
}


void mixMotors() 
{ 
  if (!armed) { 
 
    FR = 1000; 
    FL = 1000; 
    RR = 1000; 
    RL = 1000; 
 
    return; 
  } 
 
  int throttle = throttle_in; 
 
  if (throttle > 1800) { 
    throttle = 1800; 
  } 
 
  FR = throttle - pitch_pid + roll_pid - yaw_pid; 
  RR = throttle + pitch_pid + roll_pid + yaw_pid; 
  RL = throttle + pitch_pid - roll_pid - yaw_pid; 
  FL = throttle - pitch_pid - roll_pid + yaw_pid; 
 
  FR = constrain(FR, 1100, 2000); 
  FL = constrain(FL, 1100, 2000); 
  RR = constrain(RR, 1100, 2000); 
  RL = constrain(RL, 1100, 2000); 
} 
 
 
void setup() { 
 
  // Setting GPIO pins 
  pinMode(LED_BUILTIN, OUTPUT); 
 
  pinMode(CH_ROLL_PIN, INPUT); 
  pinMode(CH_PITCH_PIN, INPUT); 
  pinMode(CH_THROTTLE_PIN, INPUT); 
  pinMode(CH_YAW_PIN, INPUT); 
 
  pinMode(ESC1_PIN, OUTPUT); 
  pinMode(ESC2_PIN, OUTPUT); 
  pinMode(ESC3_PIN, OUTPUT); 
  pinMode(ESC4_PIN, OUTPUT); 
 
  // Debug 
  if (DEBUG_OUTPUT){ 
    Serial.begin(250000); 
  } 


  // Receiver interrupts 
  setupReceiverInterrupts(); 
   

  // ESCs 
  FR = FL = RR = RL = 1000;  
  updateESCCommands();  

  // Start 250 Hz ESC output 
  setupESCTimer(); 
 
  delay(3000); 
 
  // MPU  
  Wire.begin(); 
 
  byte status = mpu.begin(); 
   
  while (status != 0)  
  {  
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  
    delay(100);  
  } 
 
  delay(1000);  
 
  mpu.calcOffsets(); // Note: drone must be completely still!!  
   
  // Wait until receiver is connected and throttle is low 
  readReceiver(); 

  while (!receiver_ok || throttle_in >= 1050)  
  { 
    readReceiver(); 
    delay(20); 
  } 
 
  last_time = micros(); 
} 
 
 
void loop() { 

  // Fixed 250 Hz control loop 
  while (micros() - last_time < CONTROL_LOOP_US); 

  last_time = micros(); 
  dt = 0.004;
   
  // Receiver 
  readReceiver();  
 
 
  // IMU  
  mpu.update(); 
  roll_angle  = mpu.getAngleX(); 
  pitch_angle = mpu.getAngleY(); 
 
  gyro_roll  = mpu.getGyroX(); 
  gyro_pitch = mpu.getGyroY(); 
  gyro_yaw   = mpu.getGyroZ(); 
 
  // simple low-pass filter to reduce noisy readings 
  // This helps avoid sudden random jumps (e.g. from 10 to 20 deg/s).  
  gyro_roll_input = gyro_roll_input * 0.7 + gyro_roll * 0.3; 
  gyro_pitch_input = gyro_pitch_input * 0.7 + gyro_pitch * 0.3; 
  gyro_yaw_input = gyro_yaw_input * 0.7 + gyro_yaw * 0.3; 
 
 
  // Arm Drone 
  droneArm();  
   
  calculateSetpoints(); 

  if (armed)
  {
    calculatePID(); 
  }

  else
  {
    resetPID();
  }
 
  mixMotors(); 
 
  updateESCCommands(); 
 
  debugOutput(); 
} 


// ----Receiver Pin Interrupt---- 
ISR(PCINT0_vect) 
{ 
  unsigned long now = micros(); 

  // D9 - Roll 
  if (PINB & B00000010) 
  { 
    if (!roll_high) 
    { 
      roll_high = true; 
      roll_timer = now; 
    } 
  } 
  else if (roll_high) 
  { 
    roll_high = false; 

    unsigned long pulse = now - roll_timer; 

    if (pulse >= 900 && pulse <= 2100) 
    { 
      receiver_roll = pulse; 
      roll_last_update = now; 
    } 
  } 


  // D10 - Pitch 
  if (PINB & B00000100) 
  { 
    if (!pitch_high) 
    { 
      pitch_high = true; 
      pitch_timer = now; 
    } 
  } 
  else if (pitch_high) 
  { 
    pitch_high = false; 

    unsigned long pulse = now - pitch_timer; 

    if (pulse >= 900 && pulse <= 2100) 
    { 
      receiver_pitch = pulse; 
      pitch_last_update = now; 
    } 
  } 


  // D11 - Throttle 
  if (PINB & B00001000) 
  { 
    if (!throttle_high) 
    { 
      throttle_high = true; 
      throttle_timer = now; 
    } 
  } 
  else if (throttle_high) 
  { 
    throttle_high = false; 

    unsigned long pulse = now - throttle_timer; 

    if (pulse >= 900 && pulse <= 2100) 
    { 
      receiver_throttle = pulse; 
      throttle_last_update = now; 
    } 
  } 


  // D12 - Yaw 
  if (PINB & B00010000) 
  { 
    if (!yaw_high) 
    { 
      yaw_high = true; 
      yaw_timer = now; 
    } 
  } 
  else if (yaw_high) 
  { 
    yaw_high = false; 

    unsigned long pulse = now - yaw_timer; 

    if (pulse >= 900 && pulse <= 2100) 
    { 
      receiver_yaw = pulse; 
      yaw_last_update = now; 
    } 
  } 
} 


// ----250 Hz ESC Timer Interrupt---- 
ISR(TIMER1_COMPA_vect) 
{ 
  static bool waiting_for_frame = true; 

  static uint16_t fr_end = 0; 
  static uint16_t fl_end = 0; 
  static uint16_t rr_end = 0; 
  static uint16_t rl_end = 0; 

  if (waiting_for_frame) 
  { 
    // Start new 4 ms frame 
    TCNT1 = 0; 

    // D3-D6 HIGH at the same time 
    PORTD |= B01111000; 

    // Timer tick = 0.5 us, so pulse width is multiplied by 2 
    fr_end = esc_FR * 2; 
    fl_end = esc_FL * 2; 
    rr_end = esc_RR * 2; 
    rl_end = esc_RL * 2; 

    // Find first motor pulse that needs to end 
    uint16_t next_event = fr_end; 

    if (fl_end < next_event) next_event = fl_end; 
    if (rr_end < next_event) next_event = rr_end; 
    if (rl_end < next_event) next_event = rl_end; 

    OCR1A = next_event; 

    waiting_for_frame = false; 
  } 

  else 
  { 
    uint16_t now = TCNT1; 

    // End each ESC pulse when its commanded width is reached 
    if (now >= fr_end) PORTD &= ~B00001000; // D3 - Front Right 
    if (now >= fl_end) PORTD &= ~B00010000; // D4 - Front Left 
    if (now >= rr_end) PORTD &= ~B00100000; // D5 - Rear Right 
    if (now >= rl_end) PORTD &= ~B01000000; // D6 - Rear Left 


    // All ESC pulses finished 
    if ((PORTD & B01111000) == 0) 
    { 
      // 8000 timer ticks = 4000 us = 250 Hz 
      OCR1A = 8000; 

      waiting_for_frame = true; 
    } 

    else 
    { 
      // Find next ESC pulse that needs to end 
      uint16_t next_event = 8000; 

      if ((PORTD & B00001000) && fr_end < next_event) next_event = fr_end; 
      if ((PORTD & B00010000) && fl_end < next_event) next_event = fl_end; 
      if ((PORTD & B00100000) && rr_end < next_event) next_event = rr_end; 
      if ((PORTD & B01000000) && rl_end < next_event) next_event = rl_end; 

      OCR1A = next_event; 
    } 
  } 
}  

