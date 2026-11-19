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
  uint32_t last_us = 0;        // arrival time of the last sample (for the rate estimate)
  int rate_good = 0;           // consecutive clean samples since the last data gap
  bool reanchor = false;       // world frame restarted: lock target must be carried over
  Mat3 R_before = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};   // last attitude in the old frame
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
  uint32_t fail_streak = 0;    // consecutive failed reads
  int volt_dv = 0;             // supply voltage seen by the servo [0.1 V]
  int volt_min_dv = 255;       // lowest voltage since the last health line
  int temp_c = 0;
};
