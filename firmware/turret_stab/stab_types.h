// Types live in a header so the Arduino preprocessor's auto-generated
// function prototypes (inserted before the first function in the .ino)
// can see them.
#pragma once
#include <Arduino.h>
#include "SparkFun_BNO080_Arduino_Library.h"

struct Vec3 { float x, y, z; };
struct Mat3 { float m[3][3]; };
struct Quat { float w, x, y, z; };

struct ImuState {
  BNO080 dev;
  uint8_t addr = 0;
  const char *name = "";
  bool ready = false;
  bool has_data = false;
  bool fresh = false;          // a new sample arrived since the controller last used it
  Quat q = {1, 0, 0, 0};
  Quat q_prev = {1, 0, 0, 0};
  Mat3 R = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};   // body -> world
  Vec3 w_body = {0, 0, 0};     // filtered angular rate in body frame [rad/s]
  uint32_t last_ms = 0;
  uint32_t count = 0;          // samples since last health line
  uint32_t resets = 0;
};

struct ServoState {
  uint8_t id;
  int zero_tick;
  int dir;
  float min_deg, max_deg;
  float deg_per_tick;
  float cmd_deg = 0;
  int cmd_tick = -1;
  int meas_tick = -1;
  int meas_load = 0;
  uint32_t reads_ok = 0, reads_fail = 0;
};
