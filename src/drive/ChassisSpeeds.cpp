#include "drive/ChassisSpeeds.h"

ChassisSpeeds::ChassisSpeeds(double vx, double vy, double omega)
        : vx(vx), vy(vy), omega(omega) {}

double ChassisSpeeds::getVx() const {
        return vx;
}

double ChassisSpeeds::getVy() const {
        return vy;
}

double ChassisSpeeds::getOmega() const {
        return omega;
}

//
ChassisSpeeds ChassisSpeeds::fromFieldOrientedSpeeds(double fieldVx, double fieldVy, double fieldOmega, double robotOrientation) {
        double cosA = cos(-robotOrientation * M_PI / 180);
        double sinA = sin(-robotOrientation * M_PI / 180);
        double robotVx = fieldVx * cosA + fieldVy * sinA;
        double robotVy = -fieldVx * sinA + fieldVy * cosA;
        return ChassisSpeeds(robotVx, robotVy, fieldOmega);
}