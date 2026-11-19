/*
 * ============================================================================
 * turret_probe — Phase 0 diagnostic firmware (NO stabilization control)
 * ENR 312 — 2-axis inertially stabilized turret, ESP32-C6-DevKitC-1
 * ============================================================================
 * Purpose: observe how the hardware actually behaves before writing a
 * controller. It streams both IMU orientations, the servos' MEASURED positions
 * and loads, and the turret joint angles reconstructed from the two IMUs.
 * Servos only move when you command them.
 *
 * Output lines (921600 baud; with native USB the baud rate is ignored):
 *   H,...   CSV header for the D lines
 *   D,...   one data sample (see printHeader() for the columns)
 *   S,...   1 Hz health line: IMU rates, servo read success, loop timing
 *   E,...   event / marker (BOOT button press or 'mark <text>')
 *   # ...   human-readable message
 *
 * Angle conventions (all in degrees, every frame is +X right, +Y forward, +Z up):
 *   yaw   + = nose turns LEFT  (CCW seen from above)   -> same sense as +azimuth
 *   pitch + = nose UP                                   -> same sense as +elevation
 *   roll  + = RIGHT side dips down (right-hand rotation about +Y forward)
 *   Euler order Z(yaw) -> X(pitch) -> Y(roll), relative to the pose at the last 'z' (tare).
 *
 * imu_az / imu_el = turret joint angles measured purely by the IMUs:
 *   R_joint = R_base^T * R_offset * R_barrel, where R_offset is captured at
 *   tare so that R_joint equals the servo-reported angles at that moment.
 *   Comparing them with the servo ticks checks the signs, the deg-per-tick
 *   scale, and the drift between the two IMU yaw references.
 *
 * Commands (type a line ending with Enter):
 *   help                 list commands
 *   stream on|off        start / stop D lines
 *   rate <hz>            D-line rate, 1..200 (default 50)
 *   z                    tare: zero both IMUs' Euler angles + capture joint offset
 *   c                    center servos to AZ=0, EL=0
 *   az <deg> / el <deg>  move one axis (degrees, clamped to limits)
 *   go <az> <el>         move both axes
 *   taz <tick> / tel <tick>  raw tick command (0..1023), for calibration
 *   spd <n>              servo speed parameter (0 = max)
 *   on / off             servo torque on / off (off = move it by hand)
 *   i2c <khz>            change I2C clock (50 / 100 / 400)
 *   imu <ms>             IMU report period in ms (e.g. 20 = 50 Hz, 5 = 200 Hz)
 *   mark <text>          drop an event marker into the log
 *   info                 print configuration
 * ============================================================================
 */

#include <Wire.h>
#include <SCServo.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include "probe_types.h"

// ==================== HARDWARE ====================
constexpr int SERVO_TX_PIN    = 10;
constexpr int SERVO_RX_PIN    = 11;
constexpr int I2C_SDA_PIN     = 6;
constexpr int I2C_SCL_PIN     = 7;
constexpr int MARK_BUTTON_PIN = 9;     // onboard BOOT button
constexpr int IMU_RESET_PIN   = 2;     // shared RST (active LOW) of both BNO08x boards

constexpr uint32_t USB_BAUD = 921600;

// Measured 2026-11-20 (servo_steps log): the sensor at 0x4A moves with the
// servos, so it is the BARREL IMU; 0x4B stays still, so it is the BASE IMU.
// (The legacy code had these swapped.)
constexpr uint8_t BASE_IMU_ADDRESS   = 0x4B;
constexpr uint8_t BARREL_IMU_ADDRESS = 0x4A;

// ==================== SERVO CALIBRATION (from legacy code, to be verified) ====================
constexpr uint8_t AZIMUTH_ID        = 2;
constexpr int     AZIMUTH_ZERO_TICK = 623;
constexpr int     AZIMUTH_DIR       = -1;     // measured: ticks UP turn the barrel RIGHT, so +deg (left) = ticks down
constexpr float   AZIMUTH_MIN_DEG   = -60.0f;
constexpr float   AZIMUTH_MAX_DEG   = +60.0f;

constexpr uint8_t ELEVATION_ID        = 1;
constexpr int     ELEVATION_ZERO_TICK = 878;
constexpr int     ELEVATION_DIR       = -1;
constexpr float   ELEVATION_MIN_DEG   = -9.0f;
constexpr float   ELEVATION_MAX_DEG   = +35.0f;

constexpr float DEG_PER_TICK = 210.0f / 1024.0f;   // to be verified against IMUs

// ==================== MATH ====================
constexpr float RAD2DEG = 57.29577951f;
constexpr float DEG2RAD = 0.01745329252f;

const Mat3 IDENTITY = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

Mat3 matMul(const Mat3 &A, const Mat3 &B) {
  Mat3 C;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      C.m[i][j] = A.m[i][0] * B.m[0][j] + A.m[i][1] * B.m[1][j] + A.m[i][2] * B.m[2][j];
  return C;
}

Mat3 matT(const Mat3 &A) {
  Mat3 T;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) T.m[i][j] = A.m[j][i];
  return T;
}

// Body-to-world rotation from a unit quaternion (normalized here for safety).
Mat3 quatToMat3(float w, float x, float y, float z) {
  float n = sqrtf(w * w + x * x + y * y + z * z);
  if (n < 1e-6f) return IDENTITY;
  w /= n; x /= n; y /= n; z /= n;
  Mat3 R;
  R.m[0][0] = 1 - 2 * (y * y + z * z); R.m[0][1] = 2 * (x * y - z * w);     R.m[0][2] = 2 * (x * z + y * w);
  R.m[1][0] = 2 * (x * y + z * w);     R.m[1][1] = 1 - 2 * (x * x + z * z); R.m[1][2] = 2 * (y * z - x * w);
  R.m[2][0] = 2 * (x * z - y * w);     R.m[2][1] = 2 * (y * z + x * w);     R.m[2][2] = 1 - 2 * (x * x + y * y);
  return R;
}

// Turret kinematics: azimuth about base +Z, then elevation about the yoke +X.
Mat3 turretRot(float az_deg, float el_deg) {
  float ca = cosf(az_deg * DEG2RAD), sa = sinf(az_deg * DEG2RAD);
  float ce = cosf(el_deg * DEG2RAD), se = sinf(el_deg * DEG2RAD);
  Mat3 Rz = {{{ca, -sa, 0}, {sa, ca, 0}, {0, 0, 1}}};
  Mat3 Rx = {{{1, 0, 0}, {0, ce, -se}, {0, se, ce}}};
  return matMul(Rz, Rx);
}

// Euler Z(yaw)-X(pitch)-Y(roll) for a +Y-forward body; see header for signs.
void matToYPR(const Mat3 &R, float &yaw, float &pitch, float &roll) {
  float s = R.m[2][1];
  if (s > 1) s = 1;
  if (s < -1) s = -1;
  pitch = asinf(s) * RAD2DEG;
  yaw   = atan2f(-R.m[0][1], R.m[1][1]) * RAD2DEG;
  roll  = atan2f(-R.m[2][0], R.m[2][2]) * RAD2DEG;
}

// Pointing angles of a rotation's +Y (forward) column.
void matToAzEl(const Mat3 &R, float &az, float &el) {
  float fx = R.m[0][1], fy = R.m[1][1], fz = R.m[2][1];
  az = atan2f(-fx, fy) * RAD2DEG;
  el = atan2f(fz, sqrtf(fx * fx + fy * fy)) * RAD2DEG;
}

// ==================== IMU STATE ====================
ImuState base_imu;
ImuState barrel_imu;

// ==================== SERVO STATE ====================
SCSCL servo_bus;

ServoState az_servo = {AZIMUTH_ID, AZIMUTH_ZERO_TICK, AZIMUTH_DIR, AZIMUTH_MIN_DEG, AZIMUTH_MAX_DEG};
ServoState el_servo = {ELEVATION_ID, ELEVATION_ZERO_TICK, ELEVATION_DIR, ELEVATION_MIN_DEG, ELEVATION_MAX_DEG};

uint16_t servo_speed = 1200;

float tickToDeg(const ServoState &s, int tick) { return s.dir * (tick - s.zero_tick) * DEG_PER_TICK; }
int   degToTick(const ServoState &s, float deg) { return s.zero_tick + (int)roundf(s.dir * deg / DEG_PER_TICK); }

void writeServoTick(ServoState &s, int tick) {
  s.cmd_tick = constrain(tick, 0, 1023);
  s.cmd_deg = tickToDeg(s, s.cmd_tick);
  servo_bus.WritePos(s.id, s.cmd_tick, 0, servo_speed);
}

void writeServoDeg(ServoState &s, float deg) {
  deg = constrain(deg, s.min_deg, s.max_deg);
  writeServoTick(s, degToTick(s, deg));
}

void readServo(ServoState &s) {
  if (servo_bus.FeedBack(s.id) != -1) {
    s.meas_tick  = servo_bus.ReadPos(-1);
    s.meas_speed = servo_bus.ReadSpeed(-1);
    s.meas_load  = servo_bus.ReadLoad(-1);
    s.reads_ok++;
  } else {
    s.meas_tick = -1;
    s.reads_fail++;
  }
}

// Best estimate of where the servo is: measured if available, else commanded.
float servoActualDeg(const ServoState &s) {
  return (s.meas_tick >= 0) ? tickToDeg(s, s.meas_tick) : s.cmd_deg;
}

// ==================== RUNTIME CONFIG ====================
bool     streaming     = true;
uint32_t stream_period_ms = 20;     // 50 Hz
uint32_t i2c_khz       = 50;
uint16_t imu_period_ms = 20;        // 50 Hz IMU reports
Mat3     joint_offset  = IDENTITY;  // maps barrel-world into base-world (set at tare)
bool     tared         = false;
uint32_t seq           = 0;
uint32_t loop_max_us   = 0;

// ==================== I2C / IMU SETUP ====================
// Hardware-reset both BNO08x. An ESP32 reset in the middle of an I2C read can
// leave a BNO08x clock-stretching forever (SDA and SCL both held at 0 V);
// only pulling RST low (or a power cycle) releases it.
void hardResetImus() {
  pinMode(IMU_RESET_PIN, OUTPUT);
  digitalWrite(IMU_RESET_PIN, LOW);
  delay(20);
  digitalWrite(IMU_RESET_PIN, HIGH);
  delay(300);                        // BNO08x boot time before it answers on I2C
}

void recoverI2CBus() {
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delay(5);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
  }
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delay(10);
}

void enableImuReports(ImuState &s) {
  if (s.ready) s.dev.enableGameRotationVector(imu_period_ms);
}

// List every address that ACKs on the bus (BNO08x should appear at 0x4A / 0x4B).
void scanI2C() {
  int found = 0;
  Serial.print("# I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); found++; }
  }
  Serial.printf("  (%d device%s)\n", found, found == 1 ? "" : "s");
}

// begin() soft-resets the sensor and expects the product-ID reply as the very
// next packet, which is fragile right after a reset, so retry a few times.
void beginImu(ImuState &s, uint8_t addr, const char *name) {
  s.addr = addr;
  s.name = name;
  s.ready = false;
  for (int attempt = 1; attempt <= 3 && !s.ready; attempt++) {
    s.ready = s.dev.begin(addr, Wire);
    if (!s.ready) delay(200);
    Serial.printf("# %s IMU (0x%02X) attempt %d: %s\n", name, addr, attempt, s.ready ? "OK" : "FAIL");
  }
  enableImuReports(s);
}

// Full IMU bring-up: hardware reset, bus recovery, begin + enable reports.
uint32_t imu_recoveries = 0;
void initImus() {
  hardResetImus();
  recoverI2CBus();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(i2c_khz * 1000);
  Wire.setTimeOut(100);
  beginImu(base_imu, BASE_IMU_ADDRESS, "Base");
  delay(150);
  beginImu(barrel_imu, BARREL_IMU_ADDRESS, "Barrel");
  base_imu.last_ms = barrel_imu.last_ms = millis();   // restart the staleness clock
}

// Watchdog: if either IMU has been silent for IMU_STALE_MS, reset and re-init both.
constexpr uint32_t IMU_STALE_MS = 500;
void imuWatchdog(uint32_t now) {
  static uint32_t last_attempt_ms = 0;
  if (now - last_attempt_ms < 2000) return;
  bool base_stale   = now - base_imu.last_ms   > IMU_STALE_MS;
  bool barrel_stale = now - barrel_imu.last_ms > IMU_STALE_MS;
  if (!base_stale && !barrel_stale) return;
  last_attempt_ms = now;
  imu_recoveries++;
  Serial.printf("E,%lu,imu_recover,base_stale=%d,barrel_stale=%d\n", now, base_stale, barrel_stale);
  initImus();
}

void pollImu(ImuState &s) {
  if (!s.ready) return;
  if (s.dev.hasReset()) {            // sensor rebooted (brownout, glitch): re-arm reports
    s.resets++;
    Serial.printf("# WARNING: %s IMU reported a reset, re-enabling reports\n", s.name);
    enableImuReports(s);
  }
  // Drain a few packets per loop so the SHTP queue never backs up.
  for (int i = 0; i < 4; i++) {
    uint16_t id = s.dev.getReadings();
    if (id == 0) break;
    if (id == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      s.qw = s.dev.getQuatReal();
      s.qx = s.dev.getQuatI();
      s.qy = s.dev.getQuatJ();
      s.qz = s.dev.getQuatK();
      s.accuracy = s.dev.getQuatAccuracy();
      s.R = quatToMat3(s.qw, s.qx, s.qy, s.qz);
      s.last_ms = millis();
      s.count++;
      s.has_data = true;
    }
  }
}

// ==================== TARE ====================
void tare() {
  if (!base_imu.has_data || !barrel_imu.has_data) {
    Serial.println("# Cannot tare yet: waiting for both IMUs");
    return;
  }
  base_imu.R0   = base_imu.R;
  barrel_imu.R0 = barrel_imu.R;
  // R_base * turretRot(az,el) = R_offset * R_barrel  at this instant
  Mat3 Rt = turretRot(servoActualDeg(az_servo), servoActualDeg(el_servo));
  joint_offset = matMul(matMul(base_imu.R, Rt), matT(barrel_imu.R));
  tared = true;
  Serial.printf("# TARE done at servo AZ=%.2f EL=%.2f deg\n",
                servoActualDeg(az_servo), servoActualDeg(el_servo));
  Serial.printf("E,%lu,tare\n", millis());
}

// ==================== OUTPUT ====================
void printHeader() {
  Serial.println("H,t_ms,seq,"
                 "bq_w,bq_x,bq_y,bq_z,b_acc,"
                 "rq_w,rq_x,rq_y,rq_z,r_acc,"
                 "base_yaw,base_pitch,base_roll,"
                 "brl_yaw,brl_pitch,brl_roll,"
                 "imu_az,imu_el,"
                 "cmd_az,cmd_el,meas_az,meas_el,"
                 "cmd_az_tick,cmd_el_tick,meas_az_tick,meas_el_tick,"
                 "az_speed,el_speed,az_load,el_load,"
                 "base_age_ms,brl_age_ms");
}

void printData(uint32_t now) {
  float by = 0, bp = 0, br = 0, ry = 0, rp = 0, rr = 0, jaz = 0, jel = 0;
  matToYPR(matMul(matT(base_imu.R0), base_imu.R), by, bp, br);
  matToYPR(matMul(matT(barrel_imu.R0), barrel_imu.R), ry, rp, rr);
  if (tared) {
    Mat3 Rj = matMul(matMul(matT(base_imu.R), joint_offset), barrel_imu.R);
    matToAzEl(Rj, jaz, jel);
  }
  Serial.printf("D,%lu,%lu,"
                "%.5f,%.5f,%.5f,%.5f,%u,"
                "%.5f,%.5f,%.5f,%.5f,%u,"
                "%.2f,%.2f,%.2f,"
                "%.2f,%.2f,%.2f,"
                "%.2f,%.2f,"
                "%.2f,%.2f,%.2f,%.2f,"
                "%d,%d,%d,%d,"
                "%d,%d,%d,%d,"
                "%lu,%lu\n",
                now, seq++,
                base_imu.qw, base_imu.qx, base_imu.qy, base_imu.qz, base_imu.accuracy,
                barrel_imu.qw, barrel_imu.qx, barrel_imu.qy, barrel_imu.qz, barrel_imu.accuracy,
                by, bp, br,
                ry, rp, rr,
                jaz, jel,
                az_servo.cmd_deg, el_servo.cmd_deg, servoActualDeg(az_servo), servoActualDeg(el_servo),
                az_servo.cmd_tick, el_servo.cmd_tick, az_servo.meas_tick, el_servo.meas_tick,
                az_servo.meas_speed, el_servo.meas_speed, az_servo.meas_load, el_servo.meas_load,
                now - base_imu.last_ms, now - barrel_imu.last_ms);
}

void printHealth(uint32_t now, uint32_t window_ms) {
  float scale = 1000.0f / window_ms;
  uint32_t ok = az_servo.reads_ok + el_servo.reads_ok;
  uint32_t total = ok + az_servo.reads_fail + el_servo.reads_fail;
  Serial.printf("S,%lu,base_hz=%.1f,brl_hz=%.1f,servo_ok=%lu/%lu,loop_max_us=%lu,i2c_khz=%lu,imu_ms=%u,imu_resets=%lu/%lu,recoveries=%lu\n",
                now, base_imu.count * scale, barrel_imu.count * scale, ok, total,
                loop_max_us, i2c_khz, imu_period_ms, base_imu.resets, barrel_imu.resets, imu_recoveries);
  base_imu.count = barrel_imu.count = 0;
  az_servo.reads_ok = az_servo.reads_fail = el_servo.reads_ok = el_servo.reads_fail = 0;
  loop_max_us = 0;
}

void printInfo() {
  Serial.printf("# stream=%s rate=%lu Hz | i2c=%lu kHz | imu period=%u ms | servo speed=%u\n",
                streaming ? "on" : "off", 1000 / stream_period_ms, i2c_khz, imu_period_ms, servo_speed);
  Serial.printf("# AZ id=%u zero=%d dir=%+d lim=[%.0f,%.0f] | EL id=%u zero=%d dir=%+d lim=[%.0f,%.0f] | %.5f deg/tick\n",
                az_servo.id, az_servo.zero_tick, az_servo.dir, az_servo.min_deg, az_servo.max_deg,
                el_servo.id, el_servo.zero_tick, el_servo.dir, el_servo.min_deg, el_servo.max_deg, DEG_PER_TICK);
  Serial.printf("# IMUs: base %s, barrel %s | tared=%s\n",
                base_imu.ready ? "OK" : "FAIL", barrel_imu.ready ? "OK" : "FAIL", tared ? "yes" : "no");
}

void printHelp() {
  Serial.println("# Commands: help | stream on|off | rate <hz> | z | c | az <deg> | el <deg> | go <az> <el>");
  Serial.println("#           taz <tick> | tel <tick> | spd <n> | on | off | i2c <khz> | imu <ms> | mark <text> | info");
  Serial.println("#           scan (list I2C devices) | imuinit (recover bus + retry both IMUs)");
}

// ==================== COMMANDS (non-blocking line reader) ====================
char cmd_buf[96];
size_t cmd_len = 0;

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  Serial.printf("E,%lu,cmd,%s\n", millis(), line.c_str());

  if (line == "help") printHelp();
  else if (line == "info") printInfo();
  else if (line == "stream on")  { streaming = true; printHeader(); }
  else if (line == "stream off") { streaming = false; }
  else if (line.startsWith("rate ")) {
    int hz = constrain(line.substring(5).toInt(), 1, 200);
    stream_period_ms = 1000 / hz;
    Serial.printf("# stream rate %d Hz\n", hz);
  }
  else if (line == "z") tare();
  else if (line == "c") { writeServoDeg(az_servo, 0); writeServoDeg(el_servo, 0); Serial.println("# centered"); }
  else if (line.startsWith("az ")) { writeServoDeg(az_servo, line.substring(3).toFloat()); Serial.printf("# AZ cmd %.2f deg (tick %d)\n", az_servo.cmd_deg, az_servo.cmd_tick); }
  else if (line.startsWith("el ")) { writeServoDeg(el_servo, line.substring(3).toFloat()); Serial.printf("# EL cmd %.2f deg (tick %d)\n", el_servo.cmd_deg, el_servo.cmd_tick); }
  else if (line.startsWith("go ")) {
    String rest = line.substring(3);
    rest.trim();
    int sp = rest.indexOf(' ');
    if (sp < 0) { Serial.println("# usage: go <az> <el>"); return; }
    writeServoDeg(az_servo, rest.substring(0, sp).toFloat());
    writeServoDeg(el_servo, rest.substring(sp + 1).toFloat());
    Serial.printf("# GO AZ=%.2f EL=%.2f\n", az_servo.cmd_deg, el_servo.cmd_deg);
  }
  else if (line.startsWith("taz ")) { writeServoTick(az_servo, line.substring(4).toInt()); Serial.printf("# AZ tick %d\n", az_servo.cmd_tick); }
  else if (line.startsWith("tel ")) { writeServoTick(el_servo, line.substring(4).toInt()); Serial.printf("# EL tick %d\n", el_servo.cmd_tick); }
  else if (line.startsWith("spd ")) { servo_speed = line.substring(4).toInt(); Serial.printf("# servo speed %u\n", servo_speed); }
  else if (line == "on")  { servo_bus.EnableTorque(AZIMUTH_ID, 1); servo_bus.EnableTorque(ELEVATION_ID, 1); Serial.println("# torque ON"); }
  else if (line == "off") { servo_bus.EnableTorque(AZIMUTH_ID, 0); servo_bus.EnableTorque(ELEVATION_ID, 0); Serial.println("# torque OFF (move by hand)"); }
  else if (line.startsWith("i2c ")) {
    i2c_khz = constrain(line.substring(4).toInt(), 10, 1000);
    Wire.setClock(i2c_khz * 1000);
    Serial.printf("# I2C clock %lu kHz\n", i2c_khz);
  }
  else if (line.startsWith("imu ")) {
    imu_period_ms = constrain(line.substring(4).toInt(), 2, 1000);
    enableImuReports(base_imu);
    enableImuReports(barrel_imu);
    Serial.printf("# IMU report period %u ms (%.0f Hz)\n", imu_period_ms, 1000.0f / imu_period_ms);
  }
  else if (line == "scan") scanI2C();
  else if (line == "pins") {
    // Idle I2C lines must read HIGH (pulled up by the IMU boards). LOW = held down / shorted.
    Wire.end();
    pinMode(I2C_SDA_PIN, INPUT);
    pinMode(I2C_SCL_PIN, INPUT);
    delay(2);
    int sda_ext = digitalRead(I2C_SDA_PIN), scl_ext = digitalRead(I2C_SCL_PIN);
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
    delay(2);
    int sda_pu = digitalRead(I2C_SDA_PIN), scl_pu = digitalRead(I2C_SCL_PIN);
    Serial.printf("# pins (no internal pullup): SDA=%d SCL=%d | (with internal pullup): SDA=%d SCL=%d\n",
                  sda_ext, scl_ext, sda_pu, scl_pu);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(i2c_khz * 1000);
    Wire.setTimeOut(100);
  }
  else if (line == "imuinit") { initImus(); scanI2C(); }
  else if (line.startsWith("mark")) { Serial.printf("E,%lu,mark,%s\n", millis(), line.substring(4).c_str()); }
  else { Serial.printf("# unknown command '%s'\n", line.c_str()); printHelp(); }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmd_len > 0) {
        cmd_buf[cmd_len] = 0;
        handleCommand(String(cmd_buf));
        cmd_len = 0;
      }
    } else if (cmd_len < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_len++] = ch;
    }
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(USB_BAUD);
  Serial.setTxTimeoutMs(0);   // never block the loop when no PC is reading the USB port
  delay(1500);
  Serial.println("\n# === turret_probe (Phase 0 diagnostics) ===");
  pinMode(MARK_BUTTON_PIN, INPUT_PULLUP);

  Serial1.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  servo_bus.pSerial = &Serial1;
  servo_bus.IOTimeOut = 3;   // library default is 100 ms per reply: far too long for a control loop
  delay(100);
  Serial.printf("# Ping AZ (ID %u): %s\n", AZIMUTH_ID,   servo_bus.Ping(AZIMUTH_ID)   != -1 ? "OK" : "FAIL");
  Serial.printf("# Ping EL (ID %u): %s\n", ELEVATION_ID, servo_bus.Ping(ELEVATION_ID) != -1 ? "OK" : "FAIL");
  servo_bus.EnableTorque(AZIMUTH_ID, 1);
  servo_bus.EnableTorque(ELEVATION_ID, 1);
  writeServoDeg(az_servo, 0);
  writeServoDeg(el_servo, 0);

  initImus();
  scanI2C();

  printInfo();
  printHelp();
  printHeader();
}

// ==================== LOOP ====================
void loop() {
  uint32_t t0 = micros();
  pollSerial();

  // BOOT button drops an event marker (debounced)
  static bool last_btn = HIGH;
  static uint32_t last_btn_ms = 0;
  bool btn = digitalRead(MARK_BUTTON_PIN);
  if (last_btn == HIGH && btn == LOW && millis() - last_btn_ms > 300) {
    last_btn_ms = millis();
    Serial.printf("E,%lu,button\n", millis());
  }
  last_btn = btn;

  pollImu(base_imu);
  pollImu(barrel_imu);
  imuWatchdog(millis());

  uint32_t now = millis();
  static uint32_t last_stream_ms = 0;
  if (now - last_stream_ms >= stream_period_ms) {
    last_stream_ms = now;
    readServo(az_servo);
    readServo(el_servo);
    if (streaming) printData(now);
  }

  static uint32_t last_health_ms = 0;
  if (now - last_health_ms >= 1000) {
    printHealth(now, now - last_health_ms);
    last_health_ms = now;
  }

  uint32_t dt = micros() - t0;
  if (dt > loop_max_us) loop_max_us = dt;
}
