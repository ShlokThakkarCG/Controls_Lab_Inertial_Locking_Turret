/*
 * ============================================================================
 * ENR 312 Control Engineering Laboratory — 2-Axis Inertially Stabilized Turret
 * ============================================================================
 * Institution : Ahmedabad University (Group 6)
 * Target MCU  : ESP32-C6-DevKitC-1
 * Servos      : 2x Waveshare / Feetech SC15 Serial Bus Servos (1 Mbps, 210 deg)
 * Servo Driver: Waveshare Bus Servo Adapter (A) — Jumper A (UART mode), 8 V DC
 * IMUs        : 2x 7Semi BNO08x (BNO085/BNO086 Qwiic breakout, 3.3 V logic)
 *
 * ----------------------------------------------------------------------------
 * 1. PROJECT JOURNEY & KEY ENGINEERING DECISIONS (WHY EVERYTHING IS THIS WAY)
 * ----------------------------------------------------------------------------
 * A. Why ESP32-C6-DevKitC-1 (and why ESP8266MOD was abandoned):
 *    - SC15 servos communicate at 1,000,000 bps (1 Mbps) half-duplex TTL UART.
 *    - ESP8266MOD had SCServo/INST.h typedef collisions with <c_types.h> and
 *      Serial.swap() + updateBaudRate(1000000) caused serial corruption because
 *      ESP8266 shares one full UART with USB debug output.
 *    - ESP32-C6 has native USB-Serial/JTAG for Serial Monitor (115200 baud)
 *      AND an independent hardware Serial1 routed cleanly to GPIO 10/11 (1 Mbps).
 *    - IMPORTANT IDE SETTING: If plugged into the "USB" port, set
 *      Tools -> USB CDC On Boot -> Enabled. If plugged into the "UART" port,
 *      set Tools -> USB CDC On Boot -> Disabled.
 *
 * B. Why "SparkFun BNO080 Cortex Based IMU" library (and NOT newer libraries):
 *    - Newer "Adafruit_BNO08x" and "SparkFun_BNO08x" libraries wrap CEVA's
 *      singleton sh2 C driver, which CANNOT run two BNO08x sensors at once.
 *    - The older "SparkFun BNO080 Cortex Based IMU" library (class BNO080)
 *      supports multiple independent sensor instances on the same I2C bus.
 *    - ESP32-C6 Arduino core only exposes one standard Wire bus (Wire1 is not
 *      available for general I2C), so both IMUs share Wire on GPIO 6 / GPIO 7.
 *
 * C. Why I2C at 50 kHz + recoverI2CBus():
 *    - BNO08x uses aggressive I2C clock stretching; ESP32 hardware I2C works
 *      reliably when clocked down to 50 kHz with a 100 ms timeout.
 *    - If an I2C transfer is interrupted mid-read (or a breadboard wire
 *      wiggles), a BNO08x can freeze holding SDA/SCL low across ESP32 resets.
 *      recoverI2CBus() clocks SCL 9 times and sends a STOP condition before
 *      Wire.begin() to clear a stuck slave automatically on boot.
 *    - On 7Semi BNO08x boards, PS0 and PS1 are grounded on the PCB bottom by
 *      default (I2C mode). Base IMU uses 0x4A (MOSI->GND or ADR jumper closed);
 *      Barrel IMU uses 0x4B (default, MOSI unconnected / ADR open).
 *
 * D. Why Game Rotation Vector (6-DOF) + Two Independent World Frames:
 *    - Magnetometers suffer sudden yaw jumps near servo motors, steel hardware,
 *      and 8 V power wires. Game Rotation Vector fuses accelerometer + gyro
 *      only (gravity-referenced roll/pitch, smooth gyro-integrated yaw).
 *    - Because neither IMU uses magnetic north, their yaw=0 headings are
 *      arbitrary and CANNOT be subtracted from each other directly.
 *    - Solution: Maintain two completely independent world frames:
 *        1) base_world   : captures target direction using Base IMU + servo FK
 *        2) barrel_world : captures target direction using Barrel IMU's +Y axis
 *      Base IMU drives fast Feedforward (inverse kinematics); Barrel IMU drives
 *      closed-loop PI Feedback (cross-product pointing error in barrel frame).
 *
 * E. SC15 Servo Calibration & Mechanical Notes:
 *    - SC15 is a 210-deg potentiometer servo (0..1023 ticks, big-endian SCSCL
 *      protocol, 4.8-8.4 V rated — powered at 8 V via buck converter, NEVER 12 V).
 *    - Because it has physical hard stops at 0 and 1023 ticks, there is no
 *      software register to re-encode zero past a mechanical stop.
 *    - Verify AZIMUTH_ZERO_TICK and ELEVATION_ZERO_TICK below match your final
 *      physical assembly before enabling torque at boot!
 *
 * F. Serial Monitor Commands (115200 baud, Newline):
 *    l         : Toggle LOCK / UNLOCK (also mapped to onboard BOOT GPIO 9)
 *    m1/m2/m3  : Switch ENR 312 mode (1=No Control, 2=FB Only, 3=FB+FF)
 *    az <deg>  : Unlock and move Azimuth manually
 *    el <deg>  : Unlock and move Elevation manually
 *    0 0       : Unlock and center both axes to (0, 0)
 *    r         : Read raw servo ticks (0..1023)
 *    p <val>   : Live-update proportional_gain
 *    i <val>   : Live-update integral_gain
 *    csv       : Toggle CSV telemetry output for MATLAB plotting
 *    off / on  : Disable / enable torque on both SC15 servos
 * ============================================================================
 */

#include <Wire.h>
#include <SCServo.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// ==================== 1. HARDWARE PINOUT (ESP32-C6-DevKitC-1) ====================
// Non-strapping GPIOs chosen so boot/flash/USB-JTAG are never interfered with.
constexpr int SERVO_TX_PIN    = 10;  // -> Waveshare Adapter TX (UART1 TX)
constexpr int SERVO_RX_PIN    = 11;  // -> Waveshare Adapter RX (UART1 RX)
constexpr int I2C_SDA_PIN     = 6;   // -> Both 7Semi BNO08x SDA
constexpr int I2C_SCL_PIN     = 7;   // -> Both 7Semi BNO08x SCL
constexpr int LOCK_BUTTON_PIN = 9;   // Onboard BOOT button (safe input after boot)

constexpr uint8_t BASE_IMU_ADDRESS   = 0x4A;  // Base IMU (MOSI->GND or ADR bridged)
constexpr uint8_t BARREL_IMU_ADDRESS = 0x4B;  // Barrel IMU (default, ADR open)

// ==================== 2. SC15 SERVO CALIBRATION & LIMITS ====================
// Update AZIMUTH_ZERO_TICK / ELEVATION_ZERO_TICK if you re-seat horns or
// pick a new working center tick via calibrate_sc15.py.
constexpr uint8_t AZIMUTH_ID        = 2;
constexpr int     AZIMUTH_ZERO_TICK = 623;    // Verified center tick (adjust if re-centered)
constexpr int     AZIMUTH_DIR       = +1;     // +1: +deg turns left (CCW)
constexpr float   AZIMUTH_MIN_DEG   = -60.0f; // Safe mechanical CW limit
constexpr float   AZIMUTH_MAX_DEG   = +60.0f; // Safe mechanical CCW limit

constexpr uint8_t ELEVATION_ID        = 1;
constexpr int     ELEVATION_ZERO_TICK = 878;    // Updated center tick (earlier: 897)
constexpr int     ELEVATION_DIR       = -1;     // -1: +deg tilts up (ticks decrease)
constexpr float   ELEVATION_MIN_DEG   = -9.0f;  // Safe mechanical down limit
constexpr float   ELEVATION_MAX_DEG   = +35.0f; // Safe mechanical up limit

constexpr float    DEG_PER_TICK      = 210.0f / 1024.0f; // SC15 scale (~0.20508 deg/tick)
constexpr uint16_t SERVO_SPEED       = 1200;             // Smooth fast tracking (0 = max)
constexpr unsigned long CONTROL_PERIOD_MS = 20;          // 50 Hz deterministic control loop

// ==================== 3. IMU MOUNTING & AXIS SIGNS ====================
// Both 7Semi IMUs are mounted with +Y pointing forward along the barrel/base
// (+X = Right, +Z = Up, matching BNO08x native frame in Datasheet Fig 4-1).
// If any axis moves WITH a hand disturbance instead of opposing it, flip
// the corresponding sign below to -1.0f.
float base_azimuth_sign     = +1.0f;
float base_elevation_sign   = +1.0f;
float barrel_azimuth_sign   = +1.0f;
float barrel_elevation_sign = +1.0f;

// ==================== 4. PI FEEDBACK CONTROLLER GAINS ====================
// Conservative starting gains for real hardware; tune live via 'p <val>' / 'i <val>'.
float proportional_gain = 0.60f;
float integral_gain     = 3.50f;
constexpr float INTEGRAL_LIMIT_DEG = 25.0f;  // Anti-windup clamp on integral term

// ==================== 5. LINEAR ALGEBRA TYPES ====================
struct Vec3 {
  float x, y, z;
};

struct Mat3 {
  float m[3][3];
};

// ==================== 6. GLOBAL RUNTIME STATE ====================
SCSCL servo_bus;
BNO080 base_imu;
BNO080 barrel_imu;

bool base_imu_ready   = false;
bool barrel_imu_ready = false;
bool base_has_data    = false;
bool barrel_has_data  = false;

Mat3 base_rotation   = {{{1,0,0},{0,1,0},{0,0,1}}};
Mat3 barrel_rotation = {{{1,0,0},{0,1,0},{0,0,1}}};

bool is_locked    = false;
int  control_mode = 3;      // 1 = No Control, 2 = FB Only, 3 = FB + Feedforward
bool csv_logging  = false;

float current_azimuth_deg   = 0.0f;
float current_elevation_deg = 0.0f;
float locked_azimuth_deg    = 0.0f;
float locked_elevation_deg  = 0.0f;

Vec3 base_target_world   = {0, 1, 0};  // +Y forward
Vec3 barrel_target_world = {0, 1, 0};  // +Y forward

float azimuth_integral_deg   = 0.0f;
float elevation_integral_deg = 0.0f;

unsigned long lock_start_ms = 0;

// ==================== 7. MATH & KINEMATICS HELPERS ====================
float clampFloat(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

// Convert unit quaternion (qw, qx, qy, qz) to 3x3 body-to-world rotation matrix.
Mat3 quatToMat3(float qw, float qx, float qy, float qz) {
  Mat3 R;
  R.m[0][0] = 1.0f - 2.0f * (qy * qy + qz * qz);
  R.m[0][1] = 2.0f * (qx * qy - qz * qw);
  R.m[0][2] = 2.0f * (qx * qz + qy * qw);

  R.m[1][0] = 2.0f * (qx * qy + qz * qw);
  R.m[1][1] = 1.0f - 2.0f * (qx * qx + qz * qz);
  R.m[1][2] = 2.0f * (qy * qz - qx * qw);

  R.m[2][0] = 2.0f * (qx * qz - qy * qw);
  R.m[2][1] = 2.0f * (qy * qz + qx * qw);
  R.m[2][2] = 1.0f - 2.0f * (qx * qx + qy * qy);
  return R;
}

// Transform vector from body frame to world frame: v_world = R * v_body
Vec3 matMulVec(const Mat3 &R, const Vec3 &v) {
  return {
    R.m[0][0] * v.x + R.m[0][1] * v.y + R.m[0][2] * v.z,
    R.m[1][0] * v.x + R.m[1][1] * v.y + R.m[1][2] * v.z,
    R.m[2][0] * v.x + R.m[2][1] * v.y + R.m[2][2] * v.z
  };
}

// Transform vector from world frame back to body frame: v_body = R^T * v_world
Vec3 matTransposeMulVec(const Mat3 &R, const Vec3 &v) {
  return {
    R.m[0][0] * v.x + R.m[1][0] * v.y + R.m[2][0] * v.z,
    R.m[0][1] * v.x + R.m[1][1] * v.y + R.m[2][1] * v.z,
    R.m[0][2] * v.x + R.m[1][2] * v.y + R.m[2][2] * v.z
  };
}

// Forward kinematics for +Y forward, -X left (+azimuth), +Z up (+elevation).
Vec3 turretForwardUnit(float azimuth_deg, float elevation_deg) {
  float az = azimuth_deg   * 0.0174532925f;
  float el = elevation_deg * 0.0174532925f;
  return { -cosf(el) * sinf(az), cosf(el) * cosf(az), sinf(el) };
}

// ==================== 8. SERVO CONTROL HELPERS ====================
void commandServosDeg(float azimuth_deg, float elevation_deg) {
  current_azimuth_deg   = clampFloat(azimuth_deg,   AZIMUTH_MIN_DEG,   AZIMUTH_MAX_DEG);
  current_elevation_deg = clampFloat(elevation_deg, ELEVATION_MIN_DEG, ELEVATION_MAX_DEG);

  int az_tick = AZIMUTH_ZERO_TICK   + (int)roundf(AZIMUTH_DIR   * current_azimuth_deg   / DEG_PER_TICK);
  int el_tick = ELEVATION_ZERO_TICK + (int)roundf(ELEVATION_DIR * current_elevation_deg / DEG_PER_TICK);

  az_tick = constrain(az_tick, 0, 1023);
  el_tick = constrain(el_tick, 0, 1023);

  servo_bus.WritePos(AZIMUTH_ID,   az_tick, 0, SERVO_SPEED);
  servo_bus.WritePos(ELEVATION_ID, el_tick, 0, SERVO_SPEED);
}

// ==================== 9. I2C BUS RECOVERY ====================
// Clears a BNO08x stuck holding SDA low across ESP32 software resets.
void recoverI2CBus() {
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delay(5);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(10);
  }
  // Generate manual STOP condition
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delay(10);
}

// ==================== 10. TARGET LOCK / UNLOCK ====================
void toggleLock() {
  if (!is_locked) {
    if (!base_has_data || !barrel_has_data) {
      Serial.println("Cannot LOCK yet: waiting for both IMUs to send data.");
      return;
    }
    locked_azimuth_deg   = current_azimuth_deg;
    locked_elevation_deg = current_elevation_deg;

    // Capture target vector in EACH IMU's own independent world frame
    Vec3 turret_vec     = turretForwardUnit(locked_azimuth_deg, locked_elevation_deg);
    base_target_world   = matMulVec(base_rotation, turret_vec);
    barrel_target_world = matMulVec(barrel_rotation, {0.0f, 1.0f, 0.0f});  // +Y forward

    azimuth_integral_deg   = 0.0f;
    elevation_integral_deg = 0.0f;
    lock_start_ms          = millis();
    is_locked              = true;

    Serial.printf(">>> LOCKED (Mode %d) at AZ=%.1f deg, EL=%.1f deg\n",
                  control_mode, locked_azimuth_deg, locked_elevation_deg);
    if (csv_logging) {
      Serial.println("time_ms,mode,err_az_deg,err_el_deg,total_err_deg,cmd_az_deg,cmd_el_deg");
    }
  } else {
    is_locked = false;
    Serial.println(">>> UNLOCKED (manual mode)");
  }
}

// ==================== 11. SERIAL COMMAND HANDLER ====================
void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line == "l") {
    toggleLock();
  } else if (line == "m1") {
    control_mode = 1;
    Serial.println("Mode 1: NO CONTROL (holds locked servo angle)");
  } else if (line == "m2") {
    control_mode = 2;
    Serial.println("Mode 2: FEEDBACK ONLY (Barrel IMU PI)");
  } else if (line == "m3") {
    control_mode = 3;
    Serial.println("Mode 3: FEEDFORWARD + FEEDBACK (Base IK + Barrel PI)");
  } else if (line == "csv") {
    csv_logging = !csv_logging;
    Serial.printf("CSV logging: %s\n", csv_logging ? "ON" : "OFF");
  } else if (line == "off") {
    is_locked = false;
    servo_bus.EnableTorque(AZIMUTH_ID, 0);
    servo_bus.EnableTorque(ELEVATION_ID, 0);
    Serial.println("Servo torque OFF");
  } else if (line == "on") {
    servo_bus.EnableTorque(AZIMUTH_ID, 1);
    servo_bus.EnableTorque(ELEVATION_ID, 1);
    Serial.println("Servo torque ON");
  } else if (line == "r") {
    int az_raw = (servo_bus.FeedBack(AZIMUTH_ID)   != -1) ? servo_bus.ReadPos(-1) : -1;
    int el_raw = (servo_bus.FeedBack(ELEVATION_ID) != -1) ? servo_bus.ReadPos(-1) : -1;
    Serial.printf("RAW TICKS -> AZ (ID %u): %d | EL (ID %u): %d\n",
                  AZIMUTH_ID, az_raw, ELEVATION_ID, el_raw);
  } else if (line == "0 0") {
    is_locked = false;
    commandServosDeg(0.0f, 0.0f);
    Serial.println("Centered to AZ=0, EL=0");
  } else if (line.startsWith("az ")) {
    is_locked = false;
    commandServosDeg(line.substring(3).toFloat(), current_elevation_deg);
    Serial.printf("Manual AZ=%.1f, EL=%.1f\n", current_azimuth_deg, current_elevation_deg);
  } else if (line.startsWith("el ")) {
    is_locked = false;
    commandServosDeg(current_azimuth_deg, line.substring(3).toFloat());
    Serial.printf("Manual AZ=%.1f, EL=%.1f\n", current_azimuth_deg, current_elevation_deg);
  } else if (line.startsWith("p ")) {
    proportional_gain = line.substring(2).toFloat();
    Serial.printf("proportional_gain = %.2f\n", proportional_gain);
  } else if (line.startsWith("i ")) {
    integral_gain = line.substring(2).toFloat();
    Serial.printf("integral_gain = %.2f\n", integral_gain);
  } else {
    Serial.println("Commands: l | m1 | m2 | m3 | az <deg> | el <deg> | 0 0 | r | p <val> | i <val> | csv | off | on");
  }
}

// ==================== 12. SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== ENR 312 ESP32-C6 Stabilized Turret ===");

  pinMode(LOCK_BUTTON_PIN, INPUT_PULLUP);

  // Start SC15 Servo Bus (1 Mbps on GPIO 10/11)
  Serial1.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  servo_bus.pSerial = &Serial1;
  delay(100);
  Serial.printf("Ping Azimuth   (ID %u): %s\n", AZIMUTH_ID,   servo_bus.Ping(AZIMUTH_ID)   != -1 ? "OK" : "FAIL");
  Serial.printf("Ping Elevation (ID %u): %s\n", ELEVATION_ID, servo_bus.Ping(ELEVATION_ID) != -1 ? "OK" : "FAIL");
  commandServosDeg(0.0f, 0.0f);

  // Start Dual 7Semi BNO08x IMUs (50 kHz I2C on GPIO 6/7)
  recoverI2CBus();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(50000);
  Wire.setTimeOut(100);
  delay(300);

  base_imu_ready = base_imu.begin(BASE_IMU_ADDRESS, Wire);
  Serial.printf("Base IMU   (0x4A): %s\n", base_imu_ready ? "OK" : "FAIL");
  if (base_imu_ready) base_imu.enableGameRotationVector(20);  // 50 Hz

  delay(150);

  barrel_imu_ready = barrel_imu.begin(BARREL_IMU_ADDRESS, Wire);
  Serial.printf("Barrel IMU (0x4B): %s\n", barrel_imu_ready ? "OK" : "FAIL");
  if (barrel_imu_ready) barrel_imu.enableGameRotationVector(20);  // 50 Hz

  Serial.println("Ready! Commands: l | m1 | m2 | m3 | az <deg> | el <deg> | 0 0 | r | p <val> | i <val> | csv | off | on");
}

// ==================== 13. MAIN CONTROL LOOP ====================
void loop() {
  handleSerialCommands();

  // Debounced onboard BOOT button (GPIO 9) to toggle LOCK
  static bool last_button_state = HIGH;
  static unsigned long last_button_ms = 0;
  bool button_state = digitalRead(LOCK_BUTTON_PIN);
  if (last_button_state == HIGH && button_state == LOW && (millis() - last_button_ms > 300)) {
    last_button_ms = millis();
    toggleLock();
  }
  last_button_state = button_state;

  // Poll both IMUs continuously so SHTP packets never back up
  if (base_imu_ready && base_imu.dataAvailable()) {
    base_rotation = quatToMat3(base_imu.getQuatReal(), base_imu.getQuatI(),
                               base_imu.getQuatJ(), base_imu.getQuatK());
    base_has_data = true;
  }
  if (barrel_imu_ready && barrel_imu.dataAvailable()) {
    barrel_rotation = quatToMat3(barrel_imu.getQuatReal(), barrel_imu.getQuatI(),
                                 barrel_imu.getQuatJ(), barrel_imu.getQuatK());
    barrel_has_data = true;
  }

  // Deterministic 50 Hz control update
  static unsigned long last_control_ms = 0;
  unsigned long now_ms = millis();
  if (!is_locked || (now_ms - last_control_ms < CONTROL_PERIOD_MS)) return;

  float dt = (now_ms - last_control_ms) * 0.001f;
  last_control_ms = now_ms;

  // 1. Feedforward from Base IMU (inverse kinematics in base frame, +Y forward)
  Vec3 desired_in_base = matTransposeMulVec(base_rotation, base_target_world);
  float horizontal_norm = sqrtf(desired_in_base.x * desired_in_base.x +
                                desired_in_base.y * desired_in_base.y);
  float raw_az_ff = atan2f(-desired_in_base.x, desired_in_base.y) * 57.2957795f;
  float raw_el_ff = atan2f(desired_in_base.z, horizontal_norm)    * 57.2957795f;

  float azimuth_feedforward_deg   = locked_azimuth_deg   + base_azimuth_sign   * (raw_az_ff - locked_azimuth_deg);
  float elevation_feedforward_deg = locked_elevation_deg + base_elevation_sign * (raw_el_ff - locked_elevation_deg);

  // 2. Feedback pointing error from Barrel IMU (+Y forward, -X left, +Z up)
  Vec3 desired_in_barrel = matTransposeMulVec(barrel_rotation, barrel_target_world);
  float error_azimuth_deg   = barrel_azimuth_sign   * atan2f(-desired_in_barrel.x, desired_in_barrel.y) * 57.2957795f;
  float error_elevation_deg = barrel_elevation_sign * atan2f(desired_in_barrel.z,  desired_in_barrel.y) * 57.2957795f;
  float total_error_deg     = acosf(clampFloat(desired_in_barrel.y, -1.0f, 1.0f)) * 57.2957795f;

  // PI feedback with anti-windup clamp
  azimuth_integral_deg   = clampFloat(azimuth_integral_deg   + error_azimuth_deg   * dt, -INTEGRAL_LIMIT_DEG, INTEGRAL_LIMIT_DEG);
  elevation_integral_deg = clampFloat(elevation_integral_deg + error_elevation_deg * dt, -INTEGRAL_LIMIT_DEG, INTEGRAL_LIMIT_DEG);

  float azimuth_feedback_deg   = proportional_gain * error_azimuth_deg   + integral_gain * azimuth_integral_deg;
  float elevation_feedback_deg = proportional_gain * error_elevation_deg + integral_gain * elevation_integral_deg;

  // 3. Combine feedforward + feedback according to active ENR 312 mode
  float azimuth_command_deg   = locked_azimuth_deg;
  float elevation_command_deg = locked_elevation_deg;

  if (control_mode == 2) {
    azimuth_command_deg   = locked_azimuth_deg   + azimuth_feedback_deg;
    elevation_command_deg = locked_elevation_deg + elevation_feedback_deg;
  } else if (control_mode == 3) {
    azimuth_command_deg   = azimuth_feedforward_deg   + azimuth_feedback_deg;
    elevation_command_deg = elevation_feedforward_deg + elevation_feedback_deg;
  }

  commandServosDeg(azimuth_command_deg, elevation_command_deg);

  // 4. Print live status or CSV telemetry
  static unsigned long last_print_ms = 0;
  if (csv_logging) {
    Serial.printf("%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                  now_ms - lock_start_ms, control_mode,
                  error_azimuth_deg, error_elevation_deg, total_error_deg,
                  current_azimuth_deg, current_elevation_deg);
  } else if (now_ms - last_print_ms >= 150) {
    last_print_ms = now_ms;
    Serial.printf("[M%d] Err(AZ:%+5.1f EL:%+5.1f Tot:%4.1f) | Cmd(AZ:%+5.1f EL:%+5.1f)\n",
                  control_mode, error_azimuth_deg, error_elevation_deg, total_error_deg,
                  current_azimuth_deg, current_elevation_deg);
  }
}