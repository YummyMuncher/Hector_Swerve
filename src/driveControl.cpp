#include "driveControl.h"

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

  SwerveDrive2 sdrive;
  lemlib::PID headingPID(100, 0, 80);

  double robotHeading = 0;
  double robotVelocity = 0;
  double detectionThreshold = 0;

  bool resetTracking = false;
  bool imu_backup = true;
  bool idle = false;
  bool wasZero = false;

  bool field_centric = false;

  while (true) {
    // Gets input from controller joysticks
    //translates to forward, strafe, and rotational movement
    float fwd = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    float str = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    float rcw = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // TODO: apply deadband and s curve to controller inputs

    // double robotHeading = -angleWrap(otos_data[2]+180)* M_PI/180;

    if (field_centric) {
      double temp = fwd * cos(robotHeading) + str * sin(robotHeading);
      str = -fwd * sin(robotHeading) + str * cos(robotHeading);
      fwd = temp;
    }

    sdrive.move(str, fwd, rcw,  1);

    pros::delay(10);
  }
}