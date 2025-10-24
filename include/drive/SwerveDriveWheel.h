#ifndef _SWERVE_DRIVE_WHEEL
#define _SWERVE_DRIVE_WHEEL

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "init/devices.h"
#include "serial/serial_comm.h"
#include "lemlib/api.hpp"
#include "lemlib/pid.hpp"
#include "init/utility.h"
#include <cmath>

using namespace std;

class SwerveDriveWheel {
private:
    double calculatePID(double target, double current, bool idle);
    double zeroOffset = 0.0;
    double gearratio = 24.0 / 46.0;
    bool flipped = false;
    double *prevAngle;
    

public:
    pros::Motor* motorTop;
    pros::Motor* motorBot;
    //  ASSUMPTIONS:
    //motor encoder is built-in
    //in-built motor PID should be sufficent for differential swerve

    float* rotateEncoder;
    //PID for rotation
    lemlib::PID PIDr;

    //rotation angle in deg, for PID
    double current_r;
    double target_r;

    SwerveDriveWheel(pros::Motor* motor1, pros::Motor* motor2, float* rotateEncoder, lemlib::PID &pid, double* prevAngle, float offset = 0);

    float getAngle();
    void printAngles();
    float getMagnetAngle();
    /** Zeroes the wheel by updating the offset, and returns the new offset */
    void zero(float offset = -1);

    /**
     * @param speed wheel speed in [-127,127]
     * @param angle wheel angle in degrees increasing clockwise with 0 being forward in range [0, 360)
     * @param power scalar multiplier
     */
    void move(double speed, double angle, double power);
};

#endif