/*
 * ============================================================================
 * turret_stab — v1 inertial pointing stabilizer
 * ENR 312 — 2-axis stabilized turret, ESP32-C6-DevKitC-1
 * ============================================================================
 * LOCK captures the barrel's pointing direction in space. While locked the
 * turret keeps the barrel aimed there as the base is moved by hand.
 *
 * Control structure (each stage can be switched on/off with modes):
 *
 *   base IMU ──► [lead: predict base attitude T_lead ahead using its rate]
 *            ──► [inverse kinematics: target direction in base frame → az, el]  = feedforward
 *   barrel IMU ► [pointing error in barrel frame] ──► [PI] ─────────────────── = feedback
 *   command = feedforward + feedback  → clamp to joint limits → SC15 servos
 *
 * Modes (m <n>):
 *   0 HOLD      servos hold the angles they had at lock (no control: baseline)
 *   1 FF        feedforward from the base IMU only
 *   2 FF+LEAD   feedforward with base-motion prediction
 *   3 FULL      FF + lead + barrel-IMU PI feedback   (default)
 *   4 FB        barrel-IMU PI feedback only (for comparison)
 *
 * Frames: every body frame is +X right, +Y forward, +Z up. Angles in degrees.
 * +azimuth = barrel turns LEFT, +elevation = barrel tilts UP.
 * Turret forward kinematics: R_turret(az, el) = Rz(az) * Rx(el); barrel forward = R * [0,1,0].
 *
 * Output lines (same framing as turret_probe, so tools/turret_console.py works):
 *   H,... header   D,... data   S,... 1 Hz health   E,... events   # ... messages
 *
 * Commands:
 *   l                 toggle LOCK / UNLOCK (also the BOOT button)
 *   m <0..4>          control mode (see above)
 *   aim <daz> <del>   while locked: move the target by daz/del degrees (re-aim / sign test)
 *   lead <ms>         prediction horizon for mode 2/3 (default 0 = off until tuned)
 *   kp <v> / ki <v>   feedback gains (deg per deg, 1/s)
 *   slew <dps>        max command speed while locked (default 400)
 *   accel <dps2>      max command acceleration while locked (default 4000)
 *   az <d> / el <d> / go <az> <el> / c   manual moves (unlocks first)
 *   on / off          servo torque
 *   spd <n>           manual move speed (0 = max)     sspd <n>  stabilizer servo speed (0 = max)
 *   rate <hz>         D-line rate (default 100)       stream on|off
 *   mark <text>       event marker      info      help
 * ============================================================================
 */

#include <Wire.h>
#include <SCServo.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include "stab_types.h"

// ==================== HARDWARE ====================
constexpr int SERVO_TX_PIN    = 10;
constexpr int SERVO_RX_PIN    = 11;
constexpr int I2C_SDA_PIN     = 6;
constexpr int I2C_SCL_PIN     = 7;
constexpr int LOCK_BUTTON_PIN = 9;     // onboard BOOT button
constexpr int IMU_RESET_PIN   = 2;     // shared RST (active LOW) of both BNO08x

constexpr uint32_t USB_BAUD = 921600;
constexpr uint8_t BASE_IMU_ADDRESS   = 0x4B;   // measured: 0x4B is on the base plate
constexpr uint8_t BARREL_IMU_ADDRESS = 0x4A;   // measured: 0x4A is on the barrel
constexpr uint32_t I2C_KHZ       = 400;        // 400 kHz + 5 ms reports = full 200 Hz from both
constexpr uint16_t IMU_PERIOD_MS = 5;

// ==================== SERVO CALIBRATION (see turret_probe for how each was measured) ====================
constexpr uint8_t AZIMUTH_ID           = 2;
constexpr int     AZIMUTH_ZERO_TICK    = 623;
constexpr int     AZIMUTH_DIR          = -1;
constexpr float   AZIMUTH_DEG_PER_TICK = 0.2149f;
constexpr float   AZIMUTH_MIN_DEG      = -75.0f;
constexpr float   AZIMUTH_MAX_DEG      = +110.0f;

constexpr uint8_t ELEVATION_ID           = 1;
constexpr int     ELEVATION_ZERO_TICK    = 898;
constexpr int     ELEVATION_DIR          = -1;
constexpr float   ELEVATION_DEG_PER_TICK = 0.2228f;
constexpr float   ELEVATION_MIN_DEG      = -20.0f;
constexpr float   ELEVATION_MAX_DEG      = +55.0f;

// ==================== CONTROL PARAMETERS (runtime-tunable) ====================
int   control_mode   = 3;
float lead_ms        = 0.0f;     // base-motion prediction horizon
float kp_fb          = 0.30f;    // feedback proportional gain [deg/deg]
float ki_fb          = 2.0f;     // feedback integral gain [1/s]
float fb_int_limit   = 10.0f;    // integrator clamp [deg] (FF does the big moves; FB only trims)
float slew_dps       = 400.0f;   // max command speed [deg/s] (servo peak is ~300-360)
float accel_dps2     = 4000.0f;  // max command acceleration [deg/s^2]
float az_vel = 0, el_vel = 0;    // current command speeds used by the limiter
constexpr float LEAD_MAX_DEG       = 25.0f;   // cap on the predicted base rotation
constexpr float FB_FREEZE_ERR_DEG  = 15.0f;   // stop integrating above this pointing error
float rate_tau_s     = 0.015f;   // low-pass on the differentiated base rate
uint16_t manual_speed = 1200;    // gentle manual moves
uint16_t stab_speed   = 0;       // 0 = servo maximum speed while stabilizing
constexpr uint32_t IMU_STALE_LOCK_MS = 40;    // freeze command if an IMU sample is older than this

// ==================== MATH ====================
constexpr float RAD2DEG = 57.29577951f;
constexpr float DEG2RAD = 0.01745329252f;
const Mat3 IDENTITY = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
const Vec3 FORWARD  = {0, 1, 0};

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

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
Vec3 matVec(const Mat3 &R, const Vec3 &v) {
  return {R.m[0][0] * v.x + R.m[0][1] * v.y + R.m[0][2] * v.z,
          R.m[1][0] * v.x + R.m[1][1] * v.y + R.m[1][2] * v.z,
          R.m[2][0] * v.x + R.m[2][1] * v.y + R.m[2][2] * v.z};
}
Vec3 matTVec(const Mat3 &R, const Vec3 &v) {
  return {R.m[0][0] * v.x + R.m[1][0] * v.y + R.m[2][0] * v.z,
          R.m[0][1] * v.x + R.m[1][1] * v.y + R.m[2][1] * v.z,
          R.m[0][2] * v.x + R.m[1][2] * v.y + R.m[2][2] * v.z};
}

Quat quatNorm(Quat q) {
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < 1e-6f) return {1, 0, 0, 0};
  return {q.w / n, q.x / n, q.y / n, q.z / n};
}
Quat quatConj(const Quat &q) { return {q.w, -q.x, -q.y, -q.z}; }
Quat quatMul(const Quat &a, const Quat &b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
Mat3 quatToMat3(const Quat &q) {
  float w = q.w, x = q.x, y = q.y, z = q.z;
  Mat3 R;
  R.m[0][0] = 1 - 2 * (y * y + z * z); R.m[0][1] = 2 * (x * y - z * w);     R.m[0][2] = 2 * (x * z + y * w);
  R.m[1][0] = 2 * (x * y + z * w);     R.m[1][1] = 1 - 2 * (x * x + z * z); R.m[1][2] = 2 * (y * z - x * w);
  R.m[2][0] = 2 * (x * z - y * w);     R.m[2][1] = 2 * (y * z + x * w);     R.m[2][2] = 1 - 2 * (x * x + y * y);
  return R;
}

// Rotation by angle |r| about axis r (Rodrigues), r in radians.
Mat3 expRot(const Vec3 &r) {
  float th = sqrtf(r.x * r.x + r.y * r.y + r.z * r.z);
  if (th < 1e-7f) return IDENTITY;
  float kx = r.x / th, ky = r.y / th, kz = r.z / th;
  float s = sinf(th), c = 1 - cosf(th);
  Mat3 R;
  R.m[0][0] = 1 + c * (-ky * ky - kz * kz); R.m[0][1] = -s * kz + c * kx * ky;     R.m[0][2] = s * ky + c * kx * kz;
  R.m[1][0] = s * kz + c * kx * ky;         R.m[1][1] = 1 + c * (-kx * kx - kz * kz); R.m[1][2] = -s * kx + c * ky * kz;
  R.m[2][0] = -s * ky + c * kx * kz;        R.m[2][1] = s * kx + c * ky * kz;         R.m[2][2] = 1 + c * (-kx * kx - ky * ky);
  return R;
}

Mat3 turretRot(float az_deg, float el_deg) {
  float ca = cosf(az_deg * DEG2RAD), sa = sinf(az_deg * DEG2RAD);
  float ce = cosf(el_deg * DEG2RAD), se = sinf(el_deg * DEG2RAD);
  Mat3 Rz = {{{ca, -sa, 0}, {sa, ca, 0}, {0, 0, 1}}};
  Mat3 Rx = {{{1, 0, 0}, {0, ce, -se}, {0, se, ce}}};
  return matMul(Rz, Rx);
}

// Pointing angles of a direction vector expressed in some body frame.
void dirToAzEl(const Vec3 &d, float &az, float &el) {
  az = atan2f(-d.x, d.y) * RAD2DEG;
  el = atan2f(d.z, sqrtf(d.x * d.x + d.y * d.y)) * RAD2DEG;
}

void matToYPR(const Mat3 &R, float &yaw, float &pitch, float &roll) {
  pitch = asinf(clampf(R.m[2][1], -1, 1)) * RAD2DEG;
  yaw   = atan2f(-R.m[0][1], R.m[1][1]) * RAD2DEG;
  roll  = atan2f(-R.m[2][0], R.m[2][2]) * RAD2DEG;
}

// ==================== GLOBAL STATE ====================
SCSCL servo_bus;
ImuState base_imu, barrel_imu;
ServoState az_servo = {AZIMUTH_ID, AZIMUTH_ZERO_TICK, AZIMUTH_DIR, AZIMUTH_MIN_DEG, AZIMUTH_MAX_DEG, AZIMUTH_DEG_PER_TICK};
ServoState el_servo = {ELEVATION_ID, ELEVATION_ZERO_TICK, ELEVATION_DIR, ELEVATION_MIN_DEG, ELEVATION_MAX_DEG, ELEVATION_DEG_PER_TICK};

bool  locked = false;
Vec3  target_base_world   = {0, 1, 0};   // locked direction in the base IMU's world frame
Vec3  target_barrel_world = {0, 1, 0};   // locked direction in the barrel IMU's world frame
Mat3  base_R_at_lock = IDENTITY;
Mat3  barrel_R_at_lock = IDENTITY;
float aim_az = 0, aim_el = 0;            // target offset from the lock pose (aim command)
float lock_az = 0, lock_el = 0;
uint32_t lock_ms = 0;

// latest controller signals (for logging)
float ff_az = 0, ff_el = 0;
float fb_az = 0, fb_el = 0, int_az = 0, int_el = 0;
float err_az = 0, err_el = 0, err_tot = 0;

bool     streaming = true;
uint32_t stream_period_ms = 10;   // 100 Hz
uint32_t loop_max_us = 0, lines_dropped = 0, ctrl_count = 0, imu_recoveries = 0;
bool     stale_hold = false;

// ==================== SERVOS ====================
float tickToDeg(const ServoState &s, int tick) { return s.dir * (tick - s.zero_tick) * s.deg_per_tick; }
int   degToTick(const ServoState &s, float deg) { return s.zero_tick + (int)roundf(s.dir * deg / s.deg_per_tick); }
float servoActualDeg(const ServoState &s) { return s.meas_tick >= 0 ? tickToDeg(s, s.meas_tick) : s.cmd_deg; }

void setCmd(ServoState &s, float deg) {
  s.cmd_deg = clampf(deg, s.min_deg, s.max_deg);
  s.cmd_tick = constrain(degToTick(s, s.cmd_deg), 0, 1023);
}

// One broadcast packet for both servos: no reply, ~0.2 ms on the bus.
void sendServos(uint16_t speed) {
  u8  ids[2]   = {az_servo.id, el_servo.id};
  u16 pos[2]   = {(u16)az_servo.cmd_tick, (u16)el_servo.cmd_tick};
  u16 tim[2]   = {0, 0};
  u16 spd[2]   = {speed, speed};
  servo_bus.SyncWritePos(ids, 2, pos, tim, spd);
}

void manualMove(float az, float el) {
  if (locked) { locked = false; Serial.printf("E,%lu,unlock,manual\n", millis()); }
  setCmd(az_servo, az);
  setCmd(el_servo, el);
  sendServos(manual_speed);
}

void readServo(ServoState &s) {
  if (servo_bus.FeedBack(s.id) != -1) {
    s.meas_tick = servo_bus.ReadPos(-1);
    s.meas_load = servo_bus.ReadLoad(-1);
    s.volt_dv   = servo_bus.ReadVoltage(-1);   // 0.1 V units
    s.temp_c    = servo_bus.ReadTemper(-1);
    if (s.volt_dv > 0 && s.volt_dv < s.volt_min_dv) s.volt_min_dv = s.volt_dv;
    s.reads_ok++;
    s.fail_streak = 0;
  } else {
    s.meas_tick = -1;
    s.reads_fail++;
    s.fail_streak++;
  }
}

// Servo feedback is only for logging, so it must never stall the control loop.
// When the servos stop answering (seen after hard slams: supply/bus drop-out),
// each failed read costs a timeout; back off to a slow retry, flag it, and
// unlock so the feedback integrator cannot wind up against a dead barrel.
bool servo_comm_lost = false;
void readServosSafely(uint32_t now) {
  static uint32_t last_retry_ms = 0;
  bool failing = az_servo.fail_streak >= 5 && el_servo.fail_streak >= 5;
  if (failing && now - last_retry_ms < 250) return;
  last_retry_ms = now;
  readServo(az_servo);
  readServo(el_servo);
  bool lost = az_servo.fail_streak >= 5 && el_servo.fail_streak >= 5;
  if (lost && !servo_comm_lost) {
    Serial.printf("E,%lu,servo_lost\n", now);
    Serial.println("# WARNING: servos stopped answering (power / bus?)");
    unlockBecause("servo_lost");
  } else if (!lost && servo_comm_lost) {
    Serial.printf("E,%lu,servo_back\n", now);
    Serial.println("# servos answering again");
  }
  servo_comm_lost = lost;
}

// ==================== IMUs ====================
void hardResetImus() {
  pinMode(IMU_RESET_PIN, OUTPUT);
  digitalWrite(IMU_RESET_PIN, LOW);
  delay(20);
  digitalWrite(IMU_RESET_PIN, HIGH);
  delay(300);
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

void beginImu(ImuState &s, uint8_t addr, const char *name) {
  s.addr = addr;
  s.name = name;
  s.ready = false;
  s.has_data = false;
  for (int attempt = 1; attempt <= 3 && !s.ready; attempt++) {
    s.ready = s.dev.begin(addr, Wire);
    if (!s.ready) delay(200);
  }
  Serial.printf("# %s IMU (0x%02X): %s\n", name, addr, s.ready ? "OK" : "FAIL");
  if (s.ready) s.dev.enableGameRotationVector(IMU_PERIOD_MS);
}

void unlockBecause(const char *why) {
  if (!locked) return;
  locked = false;
  Serial.printf("E,%lu,unlock,%s\n", millis(), why);
  Serial.printf("# UNLOCKED: %s\n", why);
}

void initImus() {
  hardResetImus();
  recoverI2CBus();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_KHZ * 1000);
  Wire.setTimeOut(50);
  beginImu(base_imu, BASE_IMU_ADDRESS, "Base");
  delay(100);
  beginImu(barrel_imu, BARREL_IMU_ADDRESS, "Barrel");
  base_imu.last_ms = barrel_imu.last_ms = millis();
}

// Each Game Rotation Vector sample. The rate is the quaternion difference over
// the time since the previous sample. Lead test v1 showed why this needs care:
// after a 200+ ms data gap the old code divided a 23 deg jump by 5 ms
// (-> 1250 deg/s) and the prediction threw the barrel across its range.
//   * dt = arrival interval, floored at the report period (burst reads)
//   * a gap longer than RATE_GAP_MS resets the rate and blocks prediction
//     until RATE_GOOD_SAMPLES clean samples have arrived again
//   * the raw rate is clamped to a physically plausible hand-motion limit
constexpr float    RATE_MAX_DPS      = 600.0f;
constexpr uint32_t RATE_GAP_MS       = 20;
constexpr int      RATE_GOOD_SAMPLES = 4;
void onImuSample(ImuState &s, Quat q) {
  q = quatNorm(q);
  uint32_t now_us = micros();
  if (s.has_data) {
    uint32_t gap_us = now_us - s.last_us;
    if (gap_us > RATE_GAP_MS * 1000UL) {
      s.w_body = {0, 0, 0};
      s.rate_good = 0;
    } else {
      Quat dq = quatMul(quatConj(s.q), q);        // body-frame rotation since the last sample
      if (dq.w < 0) { dq.w = -dq.w; dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z; }
      float dt = fmaxf(gap_us * 1e-6f, IMU_PERIOD_MS * 0.001f);
      Vec3 w_raw = {2 * dq.x / dt, 2 * dq.y / dt, 2 * dq.z / dt};
      float mag = sqrtf(w_raw.x * w_raw.x + w_raw.y * w_raw.y + w_raw.z * w_raw.z);
      float lim = RATE_MAX_DPS * DEG2RAD;
      if (mag > lim) { float k = lim / mag; w_raw.x *= k; w_raw.y *= k; w_raw.z *= k; }
      float a = dt / (rate_tau_s + dt);
      s.w_body.x += a * (w_raw.x - s.w_body.x);
      s.w_body.y += a * (w_raw.y - s.w_body.y);
      s.w_body.z += a * (w_raw.z - s.w_body.z);
      if (s.rate_good < RATE_GOOD_SAMPLES) s.rate_good++;
    }
  }
  s.last_us = now_us;
  s.q_prev = s.q;
  s.q = q;
  s.R = quatToMat3(q);
  s.last_ms = millis();
  s.count++;
  s.has_data = true;
  s.fresh = true;
}

void pollImu(ImuState &s) {
  if (!s.ready) return;
  if (s.dev.hasReset()) {
    s.resets++;
    Serial.printf("# WARNING: %s IMU reset itself; re-enabling reports\n", s.name);
    s.dev.enableGameRotationVector(IMU_PERIOD_MS);
    s.has_data = false;
    unlockBecause("imu_reset");            // its world frame restarted: the lock target is meaningless now
  }
  for (int i = 0; i < 4; i++) {
    uint16_t id = s.dev.getReadings();
    if (id == 0) break;
    if (id == SENSOR_REPORTID_GAME_ROTATION_VECTOR)
      onImuSample(s, {s.dev.getQuatReal(), s.dev.getQuatI(), s.dev.getQuatJ(), s.dev.getQuatK()});
  }
}

void imuWatchdog(uint32_t now) {
  static uint32_t last_attempt_ms = 0;
  if (now - last_attempt_ms < 2000) return;
  bool base_stale   = now - base_imu.last_ms   > 500;
  bool barrel_stale = now - barrel_imu.last_ms > 500;
  if (!base_stale && !barrel_stale) return;
  last_attempt_ms = now;
  imu_recoveries++;
  Serial.printf("E,%lu,imu_recover,base_stale=%d,barrel_stale=%d\n", now, base_stale, barrel_stale);
  unlockBecause("imu_recover");
  initImus();
}

// ==================== LOCK ====================
void lock() {
  if (!base_imu.has_data || !barrel_imu.has_data) {
    Serial.println("# Cannot lock: waiting for both IMUs");
    return;
  }
  readServo(az_servo);
  readServo(el_servo);
  lock_az = servoActualDeg(az_servo);
  lock_el = servoActualDeg(el_servo);
  Vec3 fwd_base = matVec(turretRot(lock_az, lock_el), FORWARD);   // barrel direction in base frame
  target_base_world   = matVec(base_imu.R, fwd_base);
  target_barrel_world = matVec(barrel_imu.R, FORWARD);
  base_R_at_lock = base_imu.R;
  barrel_R_at_lock = barrel_imu.R;
  aim_az = aim_el = 0;
  int_az = int_el = 0;
  az_vel = el_vel = 0;
  setCmd(az_servo, lock_az);
  setCmd(el_servo, lock_el);
  lock_ms = millis();
  locked = true;
  stale_hold = false;
  Serial.printf("E,%lu,lock,mode=%d,az=%.2f,el=%.2f\n", lock_ms, control_mode, lock_az, lock_el);
  Serial.printf("# LOCKED (mode %d) at AZ=%.1f EL=%.1f\n", control_mode, lock_az, lock_el);
}

// Move the locked target to (lock_az + daz, lock_el + del) as seen from the base at lock time.
// The barrel-world target moves by the same rotation, expressed through the barrel's lock pose.
void setAim(float daz, float del) {
  if (!locked) { Serial.println("# aim needs LOCK first"); return; }
  aim_az = daz;
  aim_el = del;
  Mat3 R_old = turretRot(lock_az, lock_el);
  Mat3 R_new = turretRot(lock_az + daz, lock_el + del);
  target_base_world   = matVec(base_R_at_lock, matVec(R_new, FORWARD));
  target_barrel_world = matVec(barrel_R_at_lock, matVec(matMul(matT(R_old), R_new), FORWARD));
  Serial.printf("E,%lu,aim,%.2f,%.2f\n", millis(), daz, del);
}

void toggleLock() {
  if (locked) unlockBecause("user");
  else lock();
}

// ==================== CONTROLLER ====================
void controlStep(float dt) {
  // ---- pointing error from the barrel IMU (truth metric + feedback input) ----
  Vec3 d_brl = matTVec(barrel_imu.R, target_barrel_world);       // target seen from the barrel
  err_az  = atan2f(-d_brl.x, d_brl.y) * RAD2DEG;                  // + = target is left of the barrel
  err_el  = atan2f(d_brl.z, sqrtf(d_brl.x * d_brl.x + d_brl.y * d_brl.y)) * RAD2DEG;   // + = above
  err_tot = acosf(clampf(d_brl.y, -1, 1)) * RAD2DEG;

  // ---- feedforward: where must the joints be for the base's (predicted) attitude? ----
  Mat3 Rb = base_imu.R;
  bool use_lead = (control_mode == 2 || control_mode == 3) && lead_ms > 0 && base_imu.rate_good >= RATE_GOOD_SAMPLES;
  if (use_lead) {
    float T = lead_ms * 0.001f;
    Vec3 r = {base_imu.w_body.x * T, base_imu.w_body.y * T, base_imu.w_body.z * T};
    float ang = sqrtf(r.x * r.x + r.y * r.y + r.z * r.z);
    float max_ang = LEAD_MAX_DEG * DEG2RAD;                        // never predict a wild jump
    if (ang > max_ang) { float k = max_ang / ang; r.x *= k; r.y *= k; r.z *= k; }
    Rb = matMul(Rb, expRot(r));                                    // body-frame rotation over T
  }
  dirToAzEl(matTVec(Rb, target_base_world), ff_az, ff_el);

  // ---- feedback: PI on the barrel error, mapped to joint space ----
  bool use_fb = (control_mode == 3 || control_mode == 4);
  if (use_fb) {
    float cos_el = fmaxf(cosf(el_servo.cmd_deg * DEG2RAD), 0.3f);   // az joint moves the barrel by cos(el)
    float e_az = err_az / cos_el, e_el = err_el;
    // anti-windup: do not integrate further into a saturated joint, and freeze
    // the integrator when the error is large (FF's job / a glitch) or data is old
    bool az_sat = (az_servo.cmd_deg >= az_servo.max_deg && e_az > 0) || (az_servo.cmd_deg <= az_servo.min_deg && e_az < 0);
    bool el_sat = (el_servo.cmd_deg >= el_servo.max_deg && e_el > 0) || (el_servo.cmd_deg <= el_servo.min_deg && e_el < 0);
    bool freeze = err_tot > FB_FREEZE_ERR_DEG || (millis() - barrel_imu.last_ms) > 30 || servo_comm_lost;
    if (!az_sat && !freeze) int_az = clampf(int_az + ki_fb * e_az * dt, -fb_int_limit, fb_int_limit);
    if (!el_sat && !freeze) int_el = clampf(int_el + ki_fb * e_el * dt, -fb_int_limit, fb_int_limit);
    fb_az = kp_fb * e_az + int_az;
    fb_el = kp_fb * e_el + int_el;
  } else {
    fb_az = fb_el = 0;
  }

  // ---- combine ----
  float az_cmd = lock_az, el_cmd = lock_el;          // mode 0: hold
  switch (control_mode) {
    case 1: case 2: az_cmd = ff_az;           el_cmd = ff_el;           break;
    case 3:         az_cmd = ff_az + fb_az;   el_cmd = ff_el + fb_el;   break;
    case 4:         az_cmd = lock_az + fb_az; el_cmd = lock_el + fb_el; break;
    default: break;
  }
  // Slew limit: the command may not move faster than slew_dps. The servo cannot
  // follow faster anyway, and slamming draws current spikes that knocked the
  // servo bus out in lead test v1 / emi_servo_slam.
  // Accel limit on top: the command's speed may change by at most accel_dps2*dt
  // per step. Hand motion needs <~1300 deg/s^2 (aggressive demo), so 4000 does
  // not slow tracking but rounds off jumps; volt_slam showed hard starts sag the
  // 8 V servo supply from 7.9 to 6.6 V and knock the IMUs/servos off their buses.
  auto limitAxis = [&](float target, float cur, float &vel) {
    float v_want = clampf((target - cur) / dt, -slew_dps, slew_dps);
    float dv = accel_dps2 * dt;
    vel = clampf(v_want, vel - dv, vel + dv);
    // do not overshoot: if we cannot stop before the target at this accel, brake
    float v_stop = sqrtf(2.0f * accel_dps2 * fabsf(target - cur));
    vel = clampf(vel, -v_stop, v_stop);
    return cur + vel * dt;
  };
  az_cmd = limitAxis(az_cmd, az_servo.cmd_deg, az_vel);
  el_cmd = limitAxis(el_cmd, el_servo.cmd_deg, el_vel);
  setCmd(az_servo, az_cmd);
  setCmd(el_servo, el_cmd);
  sendServos(stab_speed);
}

// ==================== OUTPUT ====================
void printHeader() {
  Serial.println("H,t_ms,mode,locked,lead_ms,"
                 "bq_w,bq_x,bq_y,bq_z,rq_w,rq_x,rq_y,rq_z,"
                 "bw_x,bw_y,bw_z,"
                 "base_yaw,base_pitch,base_roll,"
                 "ff_az,ff_el,fb_az,fb_el,"
                 "cmd_az,cmd_el,meas_az,meas_el,"
                 "err_az,err_el,err_tot,"
                 "az_load,el_load,base_age_ms,brl_age_ms");
}

void printData(uint32_t now) {
  float by = 0, bp = 0, br = 0;
  if (locked) matToYPR(matMul(matT(base_R_at_lock), base_imu.R), by, bp, br);
  char buf[480];
  int n = snprintf(buf, sizeof(buf),
                   "D,%lu,%d,%d,%.0f,"
                   "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
                   "%.1f,%.1f,%.1f,"
                   "%.2f,%.2f,%.2f,"
                   "%.2f,%.2f,%.2f,%.2f,"
                   "%.2f,%.2f,%.2f,%.2f,"
                   "%.2f,%.2f,%.2f,"
                   "%d,%d,%lu,%lu\n",
                   now, control_mode, locked ? 1 : 0, lead_ms,
                   base_imu.q.w, base_imu.q.x, base_imu.q.y, base_imu.q.z,
                   barrel_imu.q.w, barrel_imu.q.x, barrel_imu.q.y, barrel_imu.q.z,
                   base_imu.w_body.x * RAD2DEG, base_imu.w_body.y * RAD2DEG, base_imu.w_body.z * RAD2DEG,
                   by, bp, br,
                   ff_az, ff_el, fb_az, fb_el,
                   az_servo.cmd_deg, el_servo.cmd_deg, servoActualDeg(az_servo), servoActualDeg(el_servo),
                   err_az, err_el, err_tot,
                   az_servo.meas_load, el_servo.meas_load,
                   now - base_imu.last_ms, now - barrel_imu.last_ms);
  if (n > 0 && n < (int)sizeof(buf) && Serial.availableForWrite() >= n) Serial.write((const uint8_t *)buf, n);
  else lines_dropped++;
}

void printHealth(uint32_t now, uint32_t window_ms) {
  float sc = 1000.0f / window_ms;
  uint32_t ok = az_servo.reads_ok + el_servo.reads_ok;
  uint32_t tot = ok + az_servo.reads_fail + el_servo.reads_fail;
  Serial.printf("S,%lu,base_hz=%.0f,brl_hz=%.0f,ctrl_hz=%.0f,servo_ok=%lu/%lu,loop_max_us=%lu,mode=%d,locked=%d,recoveries=%lu,dropped=%lu,volt_min=%.1f/%.1f,temp=%d/%d\n",
                now, base_imu.count * sc, barrel_imu.count * sc, ctrl_count * sc, ok, tot, loop_max_us,
                control_mode, locked ? 1 : 0, imu_recoveries, lines_dropped,
                az_servo.volt_min_dv * 0.1f, el_servo.volt_min_dv * 0.1f, az_servo.temp_c, el_servo.temp_c);
  az_servo.volt_min_dv = el_servo.volt_min_dv = 255;
  base_imu.count = barrel_imu.count = ctrl_count = 0;
  az_servo.reads_ok = az_servo.reads_fail = el_servo.reads_ok = el_servo.reads_fail = 0;
  loop_max_us = 0;
  lines_dropped = 0;
}

const char *modeName(int m) {
  switch (m) {
    case 0: return "HOLD (no control)";
    case 1: return "FF (feedforward)";
    case 2: return "FF+LEAD";
    case 3: return "FULL (FF+LEAD+FB)";
    case 4: return "FB only";
    default: return "?";
  }
}

void printInfo() {
  Serial.printf("# mode %d %s | lead %.0f ms | kp %.2f ki %.2f | slew %.0f | locked %s | stream %lu Hz\n",
                control_mode, modeName(control_mode), lead_ms, kp_fb, ki_fb, slew_dps, locked ? "yes" : "no", 1000 / stream_period_ms);
  Serial.printf("# limits AZ [%.0f, %.0f] EL [%.0f, %.0f] | servo speed manual %u stab %u | IMUs base %s barrel %s\n",
                AZIMUTH_MIN_DEG, AZIMUTH_MAX_DEG, ELEVATION_MIN_DEG, ELEVATION_MAX_DEG, manual_speed, stab_speed,
                base_imu.ready ? "OK" : "FAIL", barrel_imu.ready ? "OK" : "FAIL");
}

void printHelp() {
  Serial.println("# l | m <0..4> | aim <daz> <del> | lead <ms> | kp <v> | ki <v> | az <d> | el <d> | go <az> <el> | c | on | off");
  Serial.println("# slew <dps> | accel <dps2> | spd <n> | sspd <n> | rate <hz> | stream on|off | mark <text> | info | help");
  Serial.println("# modes: 0 HOLD, 1 FF, 2 FF+LEAD, 3 FULL, 4 FB only");
}

// ==================== COMMANDS ====================
char cmd_buf[96];
size_t cmd_len = 0;

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  Serial.printf("E,%lu,cmd,%s\n", millis(), line.c_str());

  if (line == "l") toggleLock();
  else if (line.startsWith("m ")) {
    int m = line.substring(2).toInt();
    if (m < 0 || m > 4) { Serial.println("# mode must be 0..4"); return; }
    control_mode = m;
    int_az = int_el = 0;
    Serial.printf("E,%lu,mode,%d\n", millis(), m);
    Serial.printf("# mode %d: %s\n", m, modeName(m));
  }
  else if (line.startsWith("aim ")) {
    String rest = line.substring(4);
    rest.trim();
    int sp = rest.indexOf(' ');
    if (sp < 0) { Serial.println("# usage: aim <daz> <del>"); return; }
    setAim(rest.substring(0, sp).toFloat(), rest.substring(sp + 1).toFloat());
  }
  else if (line.startsWith("lead ")) { lead_ms = clampf(line.substring(5).toFloat(), 0, 300); Serial.printf("# lead %.0f ms\n", lead_ms); }
  else if (line.startsWith("slew ")) { slew_dps = clampf(line.substring(5).toFloat(), 10, 2000); Serial.printf("# slew %.0f deg/s\n", slew_dps); }
  else if (line.startsWith("accel ")) { accel_dps2 = clampf(line.substring(6).toFloat(), 100, 100000); Serial.printf("# accel %.0f deg/s^2\n", accel_dps2); }
  else if (line.startsWith("kp "))   { kp_fb = line.substring(3).toFloat(); Serial.printf("# kp %.3f\n", kp_fb); }
  else if (line.startsWith("ki "))   { ki_fb = line.substring(3).toFloat(); int_az = int_el = 0; Serial.printf("# ki %.3f\n", ki_fb); }
  else if (line == "c") manualMove(0, 0);
  else if (line.startsWith("az ")) manualMove(line.substring(3).toFloat(), el_servo.cmd_deg);
  else if (line.startsWith("el ")) manualMove(az_servo.cmd_deg, line.substring(3).toFloat());
  else if (line.startsWith("go ")) {
    String rest = line.substring(3);
    rest.trim();
    int sp = rest.indexOf(' ');
    if (sp < 0) { Serial.println("# usage: go <az> <el>"); return; }
    manualMove(rest.substring(0, sp).toFloat(), rest.substring(sp + 1).toFloat());
  }
  else if (line == "on")  { servo_bus.EnableTorque(AZIMUTH_ID, 1); servo_bus.EnableTorque(ELEVATION_ID, 1); Serial.println("# torque ON"); }
  else if (line == "off") { unlockBecause("torque_off"); servo_bus.EnableTorque(AZIMUTH_ID, 0); servo_bus.EnableTorque(ELEVATION_ID, 0); Serial.println("# torque OFF"); }
  else if (line.startsWith("spd "))  { manual_speed = line.substring(4).toInt(); Serial.printf("# manual speed %u\n", manual_speed); }
  else if (line.startsWith("sspd ")) { stab_speed = line.substring(5).toInt(); Serial.printf("# stabilizer speed %u\n", stab_speed); }
  else if (line.startsWith("rate ")) {
    int hz = constrain(line.substring(5).toInt(), 1, 200);
    stream_period_ms = 1000 / hz;
    Serial.printf("# stream %d Hz\n", hz);
  }
  else if (line == "stream on")  { streaming = true; printHeader(); }
  else if (line == "stream off") { streaming = false; }
  else if (line.startsWith("mark")) Serial.printf("E,%lu,mark,%s\n", millis(), line.substring(4).c_str());
  else if (line == "info") printInfo();
  else if (line == "help") printHelp();
  else { Serial.printf("# unknown '%s'\n", line.c_str()); printHelp(); }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmd_len > 0) { cmd_buf[cmd_len] = 0; handleCommand(String(cmd_buf)); cmd_len = 0; }
    } else if (cmd_len < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_len++] = ch;
    }
  }
}

// ==================== SETUP / LOOP ====================
void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(USB_BAUD);
  Serial.setTxTimeoutMs(5);
  delay(1500);
  Serial.println("\n# === turret_stab v1 ===");
  pinMode(LOCK_BUTTON_PIN, INPUT_PULLUP);

  Serial1.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  servo_bus.pSerial = &Serial1;
  servo_bus.IOTimeOut = 2;
  delay(100);
  Serial.printf("# Ping AZ: %s | EL: %s\n", servo_bus.Ping(AZIMUTH_ID) != -1 ? "OK" : "FAIL",
                servo_bus.Ping(ELEVATION_ID) != -1 ? "OK" : "FAIL");
  servo_bus.EnableTorque(AZIMUTH_ID, 1);
  servo_bus.EnableTorque(ELEVATION_ID, 1);
  manualMove(0, 0);

  initImus();
  printInfo();
  printHelp();
  printHeader();
}

void loop() {
  uint32_t t0 = micros();
  pollSerial();

  static bool last_btn = HIGH;
  static uint32_t last_btn_ms = 0;
  bool btn = digitalRead(LOCK_BUTTON_PIN);
  if (last_btn == HIGH && btn == LOW && millis() - last_btn_ms > 300) {
    last_btn_ms = millis();
    Serial.printf("E,%lu,button\n", millis());
    toggleLock();
  }
  last_btn = btn;

  pollImu(base_imu);
  pollImu(barrel_imu);
  uint32_t now = millis();
  imuWatchdog(now);

  // Control runs on every new base-IMU sample (200 Hz): lowest possible latency.
  if (locked && base_imu.fresh) {
    base_imu.fresh = false;
    static uint32_t last_ctrl_us = 0;
    uint32_t tu = micros();
    float dt = (last_ctrl_us == 0) ? IMU_PERIOD_MS * 0.001f : clampf((tu - last_ctrl_us) * 1e-6f, 0.001f, 0.05f);
    last_ctrl_us = tu;
    bool stale = (now - base_imu.last_ms > IMU_STALE_LOCK_MS) || (now - barrel_imu.last_ms > IMU_STALE_LOCK_MS);
    if (!stale) {
      controlStep(dt);
      ctrl_count++;
    }
  }
  if (locked && !stale_hold && now - base_imu.last_ms > IMU_STALE_LOCK_MS) {
    stale_hold = true;                    // no fresh base data: keep the last command, flag it
    Serial.printf("E,%lu,stale_hold\n", now);
  }
  if (base_imu.last_ms == now) stale_hold = false;

  static uint32_t last_stream_ms = 0;
  if (now - last_stream_ms >= stream_period_ms) {
    last_stream_ms = now;
    readServosSafely(now);
    if (streaming) printData(now);
  }

  static uint32_t last_health_ms = 0;
  if (now - last_health_ms >= 1000) {
    printHealth(now, now - last_health_ms);
    last_health_ms = now;
  }

  uint32_t dt_us = micros() - t0;
  if (dt_us > loop_max_us) loop_max_us = dt_us;
}
