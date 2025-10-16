#ifndef _DRIVE_CONTROL
#define _DRIVE_CONTROL

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS
#include "api.h"
#include "init/devices.h"
#include "init/utility.h"
// #include "init/display.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "drive/test/SwerveDrive2.h"
#include "init/devices.h"
#include <cmath>
#include <algorithm>
#include "serial/serial_comm.h"
// #include "drive/bezierFollow/bezierFollow.h"
// #include "drive/bezierFollow/CubicBezierCurve.h"
// #include "drive/bezierFollow/TrapezoidalCubicBezierCurve.h"
#include "drive/purePursuit/Timer.h"
#include "drive/purePursuit/PID.h"
// #include "macros/intakeController.h"
// #include "macros/conveyorController.h"
// #include "init/display.h"

using namespace std;

#define TRACK_LENGTH 9.4f
#define TRACK_WIDTH 11.6f

void driveControl();
void skillsDriveControl();

#endif