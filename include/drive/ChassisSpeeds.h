#ifndef CHASSIS_SPEEDS_H
#define CHASSIS_SPEEDS_H

#include <cmath>

const double MAX_SPEED = 78.67;
const double MAX_ACCEL = 70;

class ChassisSpeeds {
public:
    ChassisSpeeds(double vx, double vy, double omega);

    double getVx() const;
    double getVy() const;
    double getOmega() const;
    double vx;
    double vy;
    double omega;
    static ChassisSpeeds fromFieldOrientedSpeeds(double fieldVx, double fieldVy, double fieldOmega, double robotOrientation);


};

#endif // CHASSIS_SPEEDS_H