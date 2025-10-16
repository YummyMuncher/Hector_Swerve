#ifndef _SWERVE_DRIVE
#define _SWERVE_DRIVE

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "init/devices.h"
#include "lemlib/api.hpp"
#include "serial/serial_comm.h"
#include <cmath>
#include "init/utility.h"
#include "drive/test/SwerveDriveWheel2.h"
#include "drive/ChassisSpeeds.h"

using namespace std;

class SwerveDrive2
{
public:
    SwerveDriveWheel2 rightFront;
    SwerveDriveWheel2 leftFront;
    SwerveDriveWheel2 leftBack;
    SwerveDriveWheel2 rightBack;

    SwerveDrive2();

    void move2(double x, double y, double angle, double power);

    // anglewrapped [-180, 180], positive is clockwise, zero is straight ahead
    // +x to the right, +y is downfield, power is scalar multiplier
    void move(double x, double y, double angle, double power);
    void reset_position();
};

extern SwerveDrive2 driveTrain;


#endif