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

void SwerveDrive::move(double speed, double angle, double rotate) {
  // pros::lcd::print(1, "target : %f", angle);
  // pros::lcd::print(2, "front : %f, %f", leftFront.getAngle(), rightFront.getAngle());
  // pros::lcd::print(3, "back : %f, %f", leftBack.getAngle(), rightBack.getAngle());

  rightFront.move(speed - rotate, angle, 1);
  leftFront.move(speed + rotate, angle, 1);
  rightBack.move(speed - rotate, angle, 1);
  leftBack.move(speed + rotate, angle, 1);
}

void SwerveDrive::reset_position() {
  rightFront.zero();
  leftFront.zero();
  leftBack.zero();
  rightBack.zero();
}