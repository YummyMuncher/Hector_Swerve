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
public:
    SwerveDriveWheel rightFront;
    SwerveDriveWheel leftFront;
    SwerveDriveWheel leftBack;
    SwerveDriveWheel rightBack;

    SwerveDrive();

    /**
     * @param speed movement speed in range [-127,127]
     * @param angle movement angle in degrees where 0 is forward, increasing is clockwise, range is (-180, 180]
     * @param rotate rotational speed of robot in degrees per second, positive is clockwise
     */
    void move(double speed, double angle, double rotate);
    void reset_position();
};

extern SwerveDrive driveTrain;


#endif