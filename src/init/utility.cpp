#include "init/utility.h"

Vector2d::Vector2d(double x, double y) : x(x), y(y) {

}

/**
 * Returns sign of integer value
 */
int signum(int value) {
	return signum((double)value);
}

/**
 * Returns sign of double value
 */
int signum(double value) {
	return (value > 0) - (value < 0);
}

/**
 * Converts degrees to radians
 */
double degToRad(double degrees) {
    return degrees * M_PI / 180;
}

/**
 * Converts radians to degrees
 */
double radToDeg(double radians) {
    return radians * 180 / M_PI;
}

double angleWrap(double degrees) {
    double sanitized = lemlib::sanitizeAngle(degrees, false);
    if (sanitized > 180) {
        sanitized -= 360;
    }
    return sanitized;
}


/**
 * Calculates signed difference between two headings, true if radians
 */
double calcAngleDiff(double angle1, double angle2){
	return lemlib::angleError(angle1, angle2, false);
}

