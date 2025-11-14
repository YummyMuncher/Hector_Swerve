#include "drive/SwerveDrive.h"

// Physical constants
#define TRACK_LENGTH 9.4f
#define TRACK_WIDTH 11.6f
#define MAX_SPEED_CONSTRAINT 1

const double RADIUS = sqrt(pow(TRACK_LENGTH, 2) + pow(TRACK_WIDTH, 2));
const double rotKP = 2;
const double rotKI = 0;
const double rotKD = 7;

// cool little matrix instead of variables
lemlib::PID rightFrontPID(rotKP + 0, rotKI + 0, rotKD + 0);
lemlib::PID leftFrontPID(rotKP + 0, rotKI + 0, rotKD + 0);
lemlib::PID leftBackPID(rotKP + 0, rotKI + 0, rotKD + 0);
lemlib::PID rightBackPID(rotKP + 0, rotKI + 0, rotKD + 0);

// front wheel offset and back wheel offset
float offset = 0;
float offset2 = 0;

// front - hub
// wheels zero - marking pointing forward
SwerveDrive::SwerveDrive()
    : rightFront(&rightFrontBottomMotor, &rightFrontTopMotor,
                 &serial_data.encoder_four, rightFrontPID, offset),
      leftFront(&leftFrontBottomMotor, &leftFrontTopMotor,
                &serial_data.encoder_one, leftFrontPID, offset),
      leftBack(&leftBackBottomMotor, &leftBackTopMotor,
               &serial_data.encoder_three, leftBackPID, offset2),
      rightBack(&rightBackBottomMotor, &rightBackTopMotor,
                &serial_data.encoder_two, rightBackPID, offset2) {}

void SwerveDrive::move(double x, double y, double rotate, double power) {
  double speed = sqrt(pow(x, 2) + pow(y, 2));

  double angle;
  if (x == 0) {
    angle = y >= 0 ? 0 : 180;
  } else if (y == 0) {
    angle = x >= 0 ? 90 : 270;
  } else {
    angle = radToDeg(atan(x / y));
    if (y < 0) {
      angle += 180;
    }
  }
  angle = angleWrap(angle);
  
  pros::lcd::print(1, "target : %f", angle);
  pros::lcd::print(1, "%f\t%f", x, y); // https://www.vexforum.com/t/i-am-wondering-why-the-vex-controller-joy-sticks-arent-nice/82102/26
  // pros::lcd::print(2, "front : %f, %f", leftFront.getAngle(), rightFront.getAngle());
  // pros::lcd::print(3, "back : %f, %f", leftBack.getAngle(), rightBack.getAngle());

  double sum = 0;
  rightFront.move(speed - rotate, angle, power);
  leftFront.move(speed + rotate, angle, power);
  rightBack.move(speed - rotate, angle, power);
  leftBack.move(speed + rotate, angle, power);
  prevAngle = angle;
}

void SwerveDrive::reset_position() {
  rightFront.zero();
  leftFront.zero();
  leftBack.zero();
  rightBack.zero();
}