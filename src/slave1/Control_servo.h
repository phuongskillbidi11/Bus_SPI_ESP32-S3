// Control_servo.h - PCA9685 Servo Control for Slave1
// 3 Servos at channels: 3, 7, 13
// I2C: SCL=10, SDA=9, OE=11

#ifndef CONTROL_SERVO_H
#define CONTROL_SERVO_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== I2C PINS ====================
#define PCA9685_SCL 10
#define PCA9685_SDA 9
#define PCA9685_OE  11

// ==================== SERVO CHANNELS ====================
#define SERVO1_CHANNEL 3      // SG90
#define SERVO2_CHANNEL 7      // MG996R  
#define SERVO3_CHANNEL 13     // SG90

// ==================== SERVO PARAMETERS ====================
#define SERVO_FREQ 50

// SG90 specs (180° rotation)
#define SG90_MIN_PULSE 75     // 0° 
#define SG90_MAX_PULSE 492    // 180°
#define SG90_MIN_ANGLE 0
#define SG90_MAX_ANGLE 180

// MG996R specs (120° rotation, center at 1.5ms)
#define MG996R_MIN_PULSE 205   // 1.0ms = 0° (left limit)
#define MG996R_CENTER_PULSE 307 // 1.5ms = 90° (CENTER - DỪNG LẠI)
#define MG996R_MAX_PULSE 410   // 2.0ms = 180° (right limit)
#define MG996R_MIN_ANGLE 0
#define MG996R_MAX_ANGLE 180
// ==================== GENERIC SERVO PARAMETERS ====================
enum ServoType {
  TYPE_SG90 = 0,
  TYPE_MG996R = 1
};
enum ServoMode {
  MODE_POSITION = 0,  // Standard servo (góc)
  MODE_CONTINUOUS = 1 // Continuous servo (tốc độ)
};
// ==================== GLOBAL VARIABLES ====================
static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
static bool servo_initialized = false;
static bool servo_reverse[3] = {false, false, false};
static float servo_current_angle[3] = {90.0, 90.0, 90.0}; // Center positions
static ServoType servo_types[3] = {TYPE_SG90, TYPE_MG996R, TYPE_SG90};
static ServoMode servo_modes[3] = {MODE_POSITION, MODE_CONTINUOUS, MODE_POSITION};
// ==================== ANGLE TO PWM CONVERSION ====================
uint16_t angleToPWM(uint8_t servo_num, float value) {
  ServoType type = servo_types[servo_num - 1];
  ServoMode mode = servo_modes[servo_num - 1];
  
  uint16_t min_pulse, max_pulse, center_pulse;
  float min_value, max_value;
  
  if (mode == MODE_CONTINUOUS) {
    // CONTINUOUS: value = speed (-100 to +100)
    center_pulse = 307;  // 1.5ms STOP
    min_pulse = 205;     // 1.0ms CW full
    max_pulse = 410;     // 2.0ms CCW full
    
    // Clamp speed
    if (value < -100) value = -100;
    if (value > 100) value = 100;
    
    // Map: -100 → 205, 0 → 307, +100 → 410
    if (value < 0) {
      // Negative: map -100..0 to 205..307
      return center_pulse + (value / 100.0) * (center_pulse - min_pulse);
    } else {
      // Positive: map 0..100 to 307..410
      return center_pulse + (value / 100.0) * (max_pulse - center_pulse);
    }
  }
  else {
    // POSITION: value = angle (0-180)
    if (type == TYPE_SG90) {
      min_pulse = SG90_MIN_PULSE;
      max_pulse = SG90_MAX_PULSE;
      min_value = SG90_MIN_ANGLE;
      max_value = SG90_MAX_ANGLE;
    } else {
      min_pulse = MG996R_MIN_PULSE;
      max_pulse = MG996R_MAX_PULSE;
      min_value = MG996R_MIN_ANGLE;
      max_value = MG996R_MAX_ANGLE;
    }
    
    if (value < min_value) value = min_value;
    if (value > max_value) value = max_value;
    
    return min_pulse + ((value - min_value) / (max_value - min_value)) * 
                       (max_pulse - min_pulse);
  }
}
// ==================== STOP ALL SERVOS ====================
void ServoStopAll() {
  if (!servo_initialized) return;
  
  // Turn off all servo channels (set PWM to 0)
  pwm.setPWM(SERVO1_CHANNEL, 0, 0);
  pwm.setPWM(SERVO2_CHANNEL, 0, 0);
  pwm.setPWM(SERVO3_CHANNEL, 0, 0);
  
  Serial.println("[SERVO] All servos stopped");
  Serial.println("[SERVO] Ready! Use serial commands to calibrate.");
  Serial.println("[SERVO] Servo Types:");
  Serial.println("[SERVO]   - Servo 1 (CH3): SG90 (0-180°)");
 Serial.println("[SERVO]   - Servo 2 (CH7): MG996R Continuous (speed -100 to +100)"); ; 
  Serial.println("[SERVO]   - Servo 3 (CH13): SG90 (0-180°)");
  Serial.println("[SERVO] Commands: s1:90, s2:60, s3:0, stop, status\n");
}
// ==================== INITIALIZE PCA9685 ====================
bool ServoInit() {
  Serial.println("\n[SERVO] Initializing PCA9685...");
  
  // Setup I2C pins
  Wire.begin(PCA9685_SDA, PCA9685_SCL);
  
  // Setup OE pin (Output Enable - active LOW)
  pinMode(PCA9685_OE, OUTPUT);
  digitalWrite(PCA9685_OE, LOW); // Enable outputs
  
  // Initialize PCA9685
  pwm.begin();
  
  // Set PWM frequency to 50Hz
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(10); // Small delay for PCA9685 to stabilize
  
  servo_initialized = true;
  
  Serial.println("[SERVO] ✓ PCA9685 initialized");
  Serial.printf("[SERVO] I2C: SDA=%d, SCL=%d, OE=%d\n", PCA9685_SDA, PCA9685_SCL, PCA9685_OE);
  Serial.printf("[SERVO] Frequency: %dHz\n", SERVO_FREQ);
  Serial.printf("[SERVO] Channels: %d, %d, %d\n", SERVO1_CHANNEL, SERVO2_CHANNEL, SERVO3_CHANNEL);
  
  // Turn off all servos initially (no auto-positioning)
  ServoStopAll();
  
  Serial.println("[SERVO] Ready! Use serial commands to calibrate.");
  Serial.println("[SERVO] Commands: s1:90, s2:45r, s3:0, stop, status\n");
  
  return true;
}

// ==================== SET SERVO ANGLE ====================
// servo_num: 1, 2, or 3
// angle: 0-180 degrees
// reverse: true = reverse direction, false = normal

bool ServoSetAngle(uint8_t servo_num, float value, bool reverse) {
  if (!servo_initialized) {
    Serial.println("[SERVO] ERROR: Not initialized!");
    return false;
  }
  
  if (servo_num < 1 || servo_num > 3) {
    Serial.println("[SERVO] ERROR: Invalid servo number (1-3)!");
    return false;
  }
  
  // Get channel
  uint8_t channel;
  switch(servo_num) {
    case 1: channel = SERVO1_CHANNEL; break;
    case 2: channel = SERVO2_CHANNEL; break;
    case 3: channel = SERVO3_CHANNEL; break;
    default: return false;
  }
  
  // Apply reverse if needed
  ServoMode mode = servo_modes[servo_num - 1];
  float final_value = value;
  
  if (reverse) {
    if (mode == MODE_CONTINUOUS) {
      final_value = -value;  // Đảo tốc độ
    } else {
      float max_angle = (servo_types[servo_num - 1] == TYPE_MG996R) ? 
                         MG996R_MAX_ANGLE : SG90_MAX_ANGLE;
      final_value = max_angle - value;  // Đảo góc
    }
  }
  
  uint16_t pwm_value = angleToPWM(servo_num, final_value);
  pwm.setPWM(channel, 0, pwm_value);
  
  // Print với đúng đơn vị
  if (mode == MODE_CONTINUOUS) {
    Serial.printf("[SERVO%d] CH%d: Speed=%.0f%% %s → PWM=%d\n", 
                  servo_num, channel, value, 
                  reverse ? "(REV)" : "", pwm_value);
  } else {
    Serial.printf("[SERVO%d] CH%d: %.1f° %s → PWM=%d\n", 
                  servo_num, channel, value, 
                  reverse ? "(REV)" : "", pwm_value);
  }
  
  servo_current_angle[servo_num - 1] = value;
  return true;
}

// ==================== SET REVERSE MODE ====================
void ServoSetReverse(uint8_t servo_num, bool reverse) {
  if (servo_num >= 1 && servo_num <= 3) {
    servo_reverse[servo_num - 1] = reverse;
    Serial.printf("[SERVO%d] Reverse mode: %s\n", 
                  servo_num, reverse ? "ON" : "OFF");
  }
}

// ==================== GET REVERSE MODE ====================
bool ServoGetReverse(uint8_t servo_num) {
  if (servo_num >= 1 && servo_num <= 3) {
    return servo_reverse[servo_num - 1];
  }
  return false;
}

// ==================== MOVE WITH STORED REVERSE SETTING ====================
bool ServoMove(uint8_t servo_num, float angle) {
  bool reverse = ServoGetReverse(servo_num);
  return ServoSetAngle(servo_num, angle, reverse);
}

// ==================== SMOOTH MOVEMENT ====================
bool ServoMoveSmooth(uint8_t servo_num, float target_angle, uint16_t duration_ms) {
  if (!servo_initialized || servo_num < 1 || servo_num > 3) return false;
  
  float start = servo_current_angle[servo_num - 1];
  float delta = target_angle - start;
  int steps = abs(delta) * 2; // 2 steps per degree
  if (steps < 10) steps = 10; // Min 10 steps
  if (steps > 100) steps = 100; // Max 100 steps
  
  float step_angle = delta / steps;
  uint16_t step_delay = duration_ms / steps;
  if (step_delay < 5) step_delay = 5; // Min 5ms
  
  for (int i = 0; i <= steps; i++) {
    float angle = start + (step_angle * i);
    ServoSetAngle(servo_num, angle, false);
    delay(step_delay);
  }
  
  servo_current_angle[servo_num - 1] = target_angle;
  return true;
}
bool ServoRotateByAngle(uint8_t servo_num, float angle_degrees, uint8_t speed_percent) {
  if (!servo_initialized || servo_num < 1 || servo_num > 3) return false;
  
  if (servo_modes[servo_num - 1] != MODE_CONTINUOUS) {
    Serial.println("[SERVO] ERROR: This function is for continuous servos only!");
    return false;
  }
  
  // CALIBRATION: Đo thực tế từ test của bạn
  // rot2:180:100 x3 = 360° thực tế
  // → 540° command = 360° thực tế
  // → Factor = 360/540 = 0.667
  // → Cần tăng thời gian lên 1.5x (1/0.667)
  
  const float CALIBRATION_FACTOR = 1.5; // 540/360 = 1.5
  
  // Tốc độ cơ bản @ 100% (theo datasheet: 0.17s/60°)
  const float DEGREES_PER_SEC_AT_100 = 60.0 / 0.17; // 353°/sec (lý thuyết)
  
  // Tính thời gian (có correction factor)
  float speed_ratio = speed_percent / 100.0;
  float actual_speed = DEGREES_PER_SEC_AT_100 * speed_ratio;
  uint32_t duration_ms = (abs(angle_degrees) / actual_speed) * 1000 * CALIBRATION_FACTOR;
  
  // Xác định hướng quay
  float speed = (angle_degrees > 0) ? speed_percent : -speed_percent;
  
  Serial.printf("[SERVO%d] Rotating %.1f° at %d%% (~%lums)\n", 
                servo_num, angle_degrees, speed_percent, duration_ms);
  
  // Quay
  ServoSetAngle(servo_num, speed, false);
  delay(duration_ms);
  ServoSetAngle(servo_num, 0, false); // Dừng
  
  Serial.printf("[SERVO%d] Complete!\n", servo_num);
  return true;
}
// ==================== DEINIT ====================
void ServoDeinit() {
  if (!servo_initialized) return;
  
  ServoStopAll();
  digitalWrite(PCA9685_OE, HIGH); // Disable outputs
  
  servo_initialized = false;
  Serial.println("[SERVO] Deinitialized");
}

// ==================== HELPER: PRINT STATUS ====================
void ServoPrintStatus() {
  Serial.println("\n========== SERVO STATUS ==========");
  Serial.printf("Initialized: %s\n", servo_initialized ? "YES" : "NO");
  Serial.printf("Servo 1 (CH%d): Type=SG90 Position, Range=0-180°, Reverse=%s\n", 
            SERVO1_CHANNEL, servo_reverse[0] ? "ON" : "OFF");
  Serial.printf("Servo 2 (CH%d): Type=MG996R Continuous, Speed=-100 to +100, Reverse=%s\n",  
            SERVO2_CHANNEL, servo_reverse[1] ? "ON" : "OFF");
  Serial.printf("Servo 3 (CH%d): Type=SG90 Position, Range=0-180°, Reverse=%s\n", 
            SERVO3_CHANNEL, servo_reverse[2] ? "ON" : "OFF");
  Serial.println("==================================\n");
}
// ==================== CALIBRATION: SWEEP TEST ====================
void ServoSweep(uint8_t servo_num, float start_angle, float end_angle, uint16_t step_delay_ms) {
  if (!servo_initialized) {
    Serial.println("[SERVO] ERROR: Not initialized!");
    return;
  }
  
  Serial.printf("[SERVO%d] Sweeping from %.1f° to %.1f° (delay=%dms)\n", 
                servo_num, start_angle, end_angle, step_delay_ms);
  Serial.println("[SERVO] Press any key to stop...\n");
  
  float current = start_angle;
  bool direction = (end_angle > start_angle);
  float step = 1.0;
  
  while (true) {
    ServoMove(servo_num, current);
    delay(step_delay_ms);
    
    // Check for user interrupt
    if (Serial.available()) {
      Serial.read();
      Serial.println("\n[SERVO] Sweep stopped!");
      return;
    }
    
    // Update angle
    if (direction) {
      current += step;
      if (current >= end_angle) current = start_angle;
    } else {
      current -= step;
      if (current <= end_angle) current = start_angle;
    }
  }
}

// ==================== CALIBRATION: SET RAW PWM ====================
// Directly set PWM value for fine-tuning (bypass angle conversion)
bool ServoSetPWM(uint8_t servo_num, uint16_t pwm_value) {
  if (!servo_initialized) {
    Serial.println("[SERVO] ERROR: Not initialized!");
    return false;
  }
  
  if (servo_num < 1 || servo_num > 3) {
    Serial.println("[SERVO] ERROR: Invalid servo number (1-3)!");
    return false;
  }
  
  uint8_t channel;
  switch(servo_num) {
    case 1: channel = SERVO1_CHANNEL; break;
    case 2: channel = SERVO2_CHANNEL; break;
    case 3: channel = SERVO3_CHANNEL; break;
    default: return false;
  }
  
  pwm.setPWM(channel, 0, pwm_value);
  
  Serial.printf("[SERVO%d] CH%d: Raw PWM=%d\n", servo_num, channel, pwm_value);
  return true;
}

// ==================== CALIBRATION: TEST ALL POSITIONS ====================
void ServoTestPositions(uint8_t servo_num) {
  if (!servo_initialized) return;
  
  Serial.printf("\n[SERVO%d] Testing key positions...\n", servo_num);
  
  Serial.println("[SERVO] Position: 0° (MIN)");
  ServoSetAngle(servo_num, 0, false);
  delay(2000);
  
  Serial.println("[SERVO] Position: 45°");
  ServoSetAngle(servo_num, 45, false);
  delay(2000);
  
  Serial.println("[SERVO] Position: 90° (CENTER)");
  ServoSetAngle(servo_num, 90, false);
  delay(2000);
  
  Serial.println("[SERVO] Position: 135°");
  ServoSetAngle(servo_num, 135, false);
  delay(2000);
  
  Serial.println("[SERVO] Position: 180° (MAX)");
  ServoSetAngle(servo_num, 180, false);
  delay(2000);
  
  Serial.println("[SERVO] Test complete! Returning to 90°");
  ServoSetAngle(servo_num, 90, false);
}

// ==================== CALIBRATION: PRINT HELP ====================
void ServoPrintHelp() {
  Serial.println("\n========== SERVO CONTROL COMMANDS ==========");
  Serial.println("Basic Control:");
  Serial.println("  s<num>:<angle>    - Set servo angle (s1:90)");
  Serial.println("  s<num>:<angle>r   - Set servo reverse (s2:45r)");
  Serial.println("  stop              - Stop all servos");
  Serial.println("  status            - Show servo status");
  Serial.println("");
  Serial.println("Calibration:");
  Serial.println("  test<num>         - Test key positions (test1)");
  Serial.println("  sweep<num>        - Sweep 0-180° (sweep1)");
  Serial.println("  pwm<num>:<val>    - Set raw PWM (pwm1:300)");
  Serial.println("  rev<num>:<0|1>    - Toggle reverse (rev1:1)");
  Serial.println("");
  Serial.println("Examples:");
  Serial.println("  s1:0              - Servo 1 (SG90) to 0°");
  Serial.println("  s2:0              - Servo 2 (MG996R) STOP");
  Serial.println("  s2:50             - Servo 2 CW at 50% speed");
  Serial.println("  s2:-50            - Servo 2 CCW at 50% speed");
  Serial.println("  s2:100            - Servo 2 CW at 100% (full speed)");
  Serial.println("  s3:180r           - Servo 3 (SG90) to 180° reversed");
  Serial.println("  test1             - Test all positions on servo 1");
  Serial.println("  sweep2            - Continuous sweep on servo 2");
  Serial.println("  pwm1:400          - Set servo 1 PWM to 400");
  Serial.println("  s<num>:<angle>s   - Smooth move (s1:90s)");
  Serial.println("============================================\n");
}

#endif // CONTROL_SERVO_H