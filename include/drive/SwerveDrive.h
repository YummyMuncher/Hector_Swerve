// #ifndef _SWERVE_DRIVE
// #define _SWERVE_DRIVE

// #define PROS_USE_SIMPLE_NAMES
// #define PROS_USE_LITERALS

// #include "init/devices.h"
// #include "lemlib/api.hpp"
// #include "serial/serial_comm.h"
// #include <cmath>
// #include "init/utility.h"
// #include "drive/SwerveDriveWheel.h"
// #include "drive/ChassisSpeeds.h"

// using namespace std;

// class SwerveDrive
// {
// public:
//     SwerveDriveWheel rightFront;
//     SwerveDriveWheel leftFront;
//     SwerveDriveWheel leftBack;
//     SwerveDriveWheel rightBack;

//     SwerveDrive();
//     bool first = true;

//     // anglewrapped [-180, 180], positive is clockwise, zero is straight ahead
//     // +x to the right, +y is downfield, power is scalar multiplier
//     void test(double speed, double angle, double power, int assembly);
//     void move(double x, double y, double angle, double power);
//     void move(double x, double y, double angle, double power, bool xDrive);
//     void pidAdjust();
//     void reset_position();
// };

// extern SwerveDrive driveTrain;


// #endif