#include "driveControl.h"
#include "init/utility.h"

/**
 * Takes input from controller joysticks
 * Controls intakes and macro execution using button presses
 */
void driveControl() {
  leftFrontTopMotor.set_brake_mode(MOTOR_BRAKE_COAST);
  leftFrontBottomMotor.set_brake_mode(MOTOR_BRAKE_COAST);
  rightFrontTopMotor.set_brake_mode(MOTOR_BRAKE_COAST);
  rightFrontBottomMotor.set_brake_mode(MOTOR_BRAKE_COAST);
  leftBackTopMotor.set_brake_mode(MOTOR_BRAKE_COAST);
  leftBackBottomMotor.set_brake_mode(MOTOR_BRAKE_COAST);
  rightBackTopMotor.set_brake_mode(MOTOR_BRAKE_COAST);
  rightBackBottomMotor.set_brake_mode(MOTOR_BRAKE_COAST);

  SwerveDrive sdrive;
  // lemlib::PID headingPID(100, 0, 80);

  // double robotHeading = 0;
  // double robotVelocity = 0;
  // double detectionThreshold = 0;

  // bool resetTracking = false;
  // bool imu_backup = true;
  // bool idle = false;
  // bool wasZero = false;

  // bool field_centric = false;

  pros::delay(1000); // wait for initialization

  while (true) {
    int raw_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    int raw_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int raw_rcw = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // raw inputs -> coords in range [-1.0, 1.0]
    // maybe divide by a smaller number (~115)
    // because the vex joysticks corners give values like
    // (-123, 123), (119, 119), (108, -108), (-114, -114)
    float scaled_x = raw_x / 127.0;
    float scaled_y = raw_y / 127.0;

    // square coords -> circle coords
    float x = scaled_x * sqrt(1 - 0.5 * pow(scaled_y, 2));
    float y = scaled_y * sqrt(1 - 0.5 * pow(scaled_x, 2));

    // circle coords -> speed in range [-127.0, 127.0] & angle
    double speed = 127 * sqrt(pow(x, 2) + pow(y, 2));
    double angle;
    if (speed == 0) {
      angle = 0;
    } else {
      angle = radToDeg(atan(x / y));
      if (y < 0) {
        angle = angleWrap(angle + 180);
      }
    }

    // if (field_centric) {
    //   double temp = fwd * cos(robotHeading) + str * sin(robotHeading);
    //   str = -fwd * sin(robotHeading) + str * cos(robotHeading);
    //   fwd = temp;
    // }

    // pros::lcd::print(1, "raw x: %f, y: %f", raw_x, raw_y);
    // pros::lcd::print(2, "x: %f, y: %f", x, y);
    // pros::lcd::print(3, "spe: %f, ang: %f", speed, angle);
    sdrive.move(speed, angle, raw_rcw);

    pros::delay(10);
  }
}