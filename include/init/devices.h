#ifndef _DEVICES
#define _DEVICES

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include <sstream>
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/serial.hpp"

// Controllers
extern pros::Controller controller;

// Drivetrain encoders
extern pros::Rotation rightFrontEncoder;
extern pros::Rotation leftFrontEncoder;
extern pros::Rotation leftBackEncoder;
extern pros::Rotation rightBackEncoder;

// Drivetrain motors & MotorGroups

// Left Front
extern pros::Motor leftFrontTopMotor;
extern pros::Motor leftFrontBottomMotor;

// Left Back
extern pros::Motor leftBackTopMotor;
extern pros::Motor leftBackBottomMotor;

// Right Front
extern pros::Motor rightFrontTopMotor;
extern pros::Motor rightFrontBottomMotor;

// Right Back
extern pros::Motor rightBackTopMotor;
extern pros::Motor rightBackBottomMotor;

// Prototypes for hidden vex functions to bypass PROS bug
extern "C" int32_t vexGenericSerialReceive(uint32_t index, uint8_t *buffer, int32_t length);
extern "C" void vexGenericSerialEnable(uint32_t index, uint32_t nu);
extern "C" void vexGenericSerialBaudrate(uint32_t index, uint32_t rate);

// Port to use for serial data
#define SERIALPORT 3

extern pros::Serial serial;

extern pros::adi::DigitalOut trackingReset;

// Intake
extern pros::Motor intake;

//IMU
extern pros::Imu imu;

#endif