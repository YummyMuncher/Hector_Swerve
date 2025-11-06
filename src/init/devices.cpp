#include "init/devices.h"

// Controllers
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drivetrain motors
// In each swerve wheel there are two motors, one motor has a gear on the top and the other has a gear on the bottom
// So leftFrontTopMotor would refer to the motor that is in the left and front of the robot and the motor who's gear is on top
// This logic extends to the other motors

// Port to use for serial data IN HEADER FILE
// pros::adi::DigitalOut trackingReset('A');

// Left Front (PORTS ARE LIKE ALL BUSTED)
pros::Motor leftFrontTopMotor(8, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor leftFrontBottomMotor(18, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

// Left Back
pros::Motor leftBackTopMotor(16, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor leftBackBottomMotor(17, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

// Right Front
pros::Motor rightFrontTopMotor(11, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor rightFrontBottomMotor(12, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

// Right Back
pros::Motor rightBackTopMotor(1, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor rightBackBottomMotor(2, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

// max485 is port 3 
pros::adi::AnalogIn max(3);

// IMU
// pros::Imu imu(15);