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
private:
    double prevAngle = 0;
public:
    SwerveDriveWheel2 rightFront;
    SwerveDriveWheel2 leftFront;
    SwerveDriveWheel2 leftBack;
    SwerveDriveWheel2 rightBack;

    SwerveDrive2();

    // x, y - translational speed of robot in [-127, 127], +x is to the right, +y is forward
    // rotate - rotational speed of robot in degrees per second, pos is clockwise
    // power - scalar multiplier
    void move(double x, double y, double rotate, double power);
    void reset_position();
};

extern SwerveDrive2 driveTrain;


#endif