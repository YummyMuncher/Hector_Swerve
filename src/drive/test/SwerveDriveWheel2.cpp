#include "drive/test/SwerveDriveWheel2.h"

// in deg
#define ANGLE_MARGIN_OF_ERROR .25
#define IDLE_ANGLE_MARGIN_OF_ERROR 8
// max velocity of motors, in RPM ticks
#define MAX_MOTOR_SPEED_TICKS

SwerveDriveWheel2::SwerveDriveWheel2(pros::Motor *motorTop,
                                     pros::Motor *motorBot,
                                     float *rotateEncoder, lemlib::PID &pid,
                                     float offset)
    : motorTop(motorTop), motorBot(motorBot),

      // float with angle in it from serial and rotation pid
      rotateEncoder(rotateEncoder), PIDr(pid) {
  current_r = getMagnetAngle();

  zero(offset);
  // zeros motor positions
  motorTop->tare_position();
  motorBot->tare_position();

  // motorTop->set_zero_position(0);
  // motorBot->set_zero_position(0);
}

float SwerveDriveWheel2::getMagnetAngle() {
  // the difference between where the wheel is and where the zero was set
  float zeroed = *rotateEncoder - zeroOffset;
  if (zeroed < 0) {
    zeroed += 1;
  } else if (zeroed > 1) {
    zeroed -= 1;
  }
  // measured in centidegrees [0, 360]
  return (((1 - zeroed) * 360));
}

float SwerveDriveWheel2::getAngle() {
  // get the current angle of the wheel in degrees
  double angleTop = motorTop->get_position();
  double angleBot = motorBot->get_position();

  // get the average of the two angles
  // (this is where the swerve wheel is facing)
  double avgAngle = (angleTop + angleBot) / 4.0f; //dividing by four for unknown reasons
  return angleWrap(avgAngle);
}

/** Zeroes the wheel by updating the offset, and returns the new offset */
void SwerveDriveWheel2::zero(float offset) {
  if (offset == -1) {
    offset = getAngle();
  }
  // set the zeroOffset to the current angle
  zeroOffset = offset;
}

double SwerveDriveWheel2::calculatePID(double target, double current,
                                       bool idle) {

  // Convert back to degrees for PID math

  // // printf("Err: %f\n Curr: %f\n", error, current);

  double error = calcAngleDiff(current, target);
  if (idle) {
    if (abs(error) < IDLE_ANGLE_MARGIN_OF_ERROR) {
      return 0;
    }
  } else {
    if (abs(error) < ANGLE_MARGIN_OF_ERROR) {
      return 0;
    }
  }

  // Calculate PID output
  double rotation = PIDr.update(error);

  if (rotation > 127) {
    rotation = 127;
  } else if (rotation < -127) {
    rotation = -127;
  }
  // Scale output to reasonable motor power
  return rotation;
}

void SwerveDriveWheel2::move(double speed, double target_angle, double maxVel) {
  float curr_angle = getAngle();

  // if (abs(curr_angle - target_angle) > 90) {
  //   flipped = !flipped;
  // }

  // if (flipped) {
  //   if (target_angle >= 0) {
  //     target_angle -= 180;
  //   } else {
  //     target_angle += 180;
  //   }
  // }

  float rotation = calculatePID(curr_angle, target_angle, false);

  motorTop->move_velocity(rotation - speed);
  motorBot->move_velocity(rotation + speed);
}