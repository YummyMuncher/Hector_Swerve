#ifndef _SWERVE_DRIVE
#define _SWERVE_DRIVE

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "init/devices.h"
#include "lemlib/api.hpp"
#include "serial/serial_comm.h"
#include <cmath>
#include "init/utility.h"
#include "drive/SwerveDriveWheel.h"
#include "drive/ChassisSpeeds.h"

using namespace std;

class SwerveDrive
{
private:
    double prevAngle = 0;
public:
    SwerveDriveWheel rightFront;
    SwerveDriveWheel leftFront;
    SwerveDriveWheel leftBack;
    SwerveDriveWheel rightBack;

    SwerveDrive();

    /**
     * @param x left/right speed of robot in range [-127, 127], positive is to the right
     * @param y forward/backward speed of robot in range [-127, 127], positive is forward
     * @param rotate rotational speed of robot in degrees per second, positive is clockwise
     * @param power scalar multiplier
     */
    void move(double x, double y, double rotate, double power);
    void reset_position();
};

extern SwerveDrive driveTrain;


#endif