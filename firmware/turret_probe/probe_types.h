// Types live in a header so the Arduino preprocessor's auto-generated
// function prototypes (inserted before the first function in the .ino)
// can see them.
#pragma once
#include <Arduino.h>
#include "SparkFun_BNO080_Arduino_Library.h"

struct Vec3 { float x, y, z; };
struct Mat3 { float m[3][3]; };

struct ImuState {
  BNO080 dev;
  uint8_t addr = 0;
  const char *name = "";
  bool ready = false;
  bool has_data = false;
  float qw = 1, qx = 0, qy = 0, qz = 0;
  uint8_t accuracy = 0;
  Mat3 R  = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};   // current body->world
  Mat3 R0 = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};   // body->world at last tare
  uint32_t last_ms = 0;
  uint32_t count = 0;    // reports since last health line
  uint32_t resets = 0;
};

struct ServoState {
  uint8_t id;
  int zero_tick;
  int dir;
  float min_deg, max_deg;
  float cmd_deg = 0;
  int cmd_tick = -1;
  int meas_tick = -1;
  int meas_speed = 0;
  int meas_load = 0;
  uint32_t reads_ok = 0, reads_fail = 0;
};
