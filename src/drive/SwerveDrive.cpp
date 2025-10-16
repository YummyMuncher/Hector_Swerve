// #include "drive/SwerveDrive.h"

// // Physical constants
// #define TRACK_LENGTH 9.4f
// #define TRACK_WIDTH 11.6f
// #define MAX_SPEED_CONSTRAINT 1

// // radius from center to each wheel
// const double RADIUS = sqrt(pow(TRACK_LENGTH, 2) + pow(TRACK_WIDTH, 2));

// //used to be const
// double rotKP = .2; // 1.37;
// double rotKI = 0;
// double rotKD = 0; // 10.75;
// int changeVar = 0;
// // const double rotKD = 0.75;

// lemlib::PID rightFrontPID(rotKP, rotKI, rotKD);
// lemlib::PID leftFrontPID(rotKP, rotKI, rotKD);
// lemlib::PID leftBackPID(rotKP, rotKI, rotKD);
// lemlib::PID rightBackPID(rotKP, rotKI, rotKD);

// SwerveDrive::SwerveDrive()
//     : rightFront(&rightFrontBottomMotor,
//                  &rightFrontTopMotor,
//                  &serial_data.encoder_four,
//                  rightFrontPID,
//                  0.01),
//       leftFront(&leftFrontBottomMotor,
//                 &leftFrontTopMotor,
//                 &serial_data.encoder_one,
//                 leftFrontPID,
//                 0.01),
//       leftBack(&leftBackBottomMotor,
//                &leftBackTopMotor,
//                &serial_data.encoder_three,
//                leftBackPID,
//                0.01),
//       rightBack(&rightBackBottomMotor,
//                 &rightBackTopMotor,
//                 &serial_data.encoder_two,
//                 rightBackPID,
//                 0.01)
// {
// }

// void SwerveDrive::pidAdjust(){
//     while(controller.get_digital(DIGITAL_L1)==1){
//         if(controller.get_digital_new_press(DIGITAL_A)){
//             changeVar++;
//         } else if(controller.get_digital_new_press(DIGITAL_Y)){
//             changeVar--;
//         }

//         if(controller.get_digital_new_press(DIGITAL_X) == 1){
//             if(changeVar==0){
//                 rotKP += .1;
//             } else if(changeVar==1){
//                 rotKI += .05;
//             } else if(changeVar == 2){
//                 rotKD += .1;
//             }
//         } else if(controller.get_digital_new_press(DIGITAL_B) == 1){
//             if(changeVar==0){
//                 rotKP -= .1;
//             } else if(changeVar==1){
//                 rotKI -= .05;
//             } else if(changeVar == 2){
//                 rotKD -= .1;
//             }
//         }
//         pros::lcd::print(5, "P: %f, I: %f, D: %f", rotKP, rotKI, rotKD);
//         pros::lcd::print(6, "changeVar: %i", changeVar);
//     }
// }

// void SwerveDrive::move(double x, double y, double angle, double power)
// {
//     move(x, y, angle, power, false);
// }

// // anglewrapped [-180, 180], positive is clockwise, zero is straight ahead
// // +x to the right, +y is downfield, power is scalar multiplier
// void SwerveDrive::move(double x, double y, double angle, double power, bool xDrive)
// {
//     double absX = abs(x);
//     double absY = abs(y);
//     double angleModifer = 0;

//     if (xDrive)
//     {
//         if (absY < 0.707 && absX < 0.707)
//         {
//             x /= 0.707;
//             y /= 0.707;
//         }
//         else
//         {
//             double scaledX = (absX - 0.707) / (1 - 0.707);
//             double scaledY = (absY - 0.707) / (1 - 0.707);
//             angleModifer = max(scaledX, scaledY) * (atan2(absX, absY) * 90 / M_PI -
//                                                     atan2(absY, absX) * 90 / M_PI);
//             // printf("angleModifer: %f\n", angleModifer);
//             // double driveangle = x/y;
//             // y = 1;
//             // x = driveangle*y;
//         }
//     }
//     double A = y - angle * TRACK_WIDTH / RADIUS;
//     double B = y + angle * TRACK_WIDTH / RADIUS;
//     double C = x - angle * TRACK_LENGTH / RADIUS;
//     double D = x + angle * TRACK_LENGTH / RADIUS;

//     // wheel 1: top right
//     // wheel 2: top left
//     // wheel 3: bottom left
//     // wheel 4: bottom right
//     double speed1 = sqrt(pow(B, 2) + pow(C, 2));
//     double angle1 = -atan2(B, C) * 180 / M_PI + angleModifer;
//     double speed2 = sqrt(pow(B, 2) + pow(D, 2));
//     double angle2 = -atan2(B, D) * 180 / M_PI - angleModifer;
//     double speed3 = sqrt(pow(A, 2) + pow(D, 2));
//     double angle3 = -atan2(A, D) * 180 / M_PI + angleModifer;
//     double speed4 = sqrt(pow(A, 2) + pow(C, 2));
//     double angle4 = -atan2(A, C) * 180 / M_PI - angleModifer;

//     // speed is in range [0, 1]
//     double maxVel = 1.0;
//     double maxSpeed =
//         max(abs(speed1), max(abs(speed2), max(abs(speed3), abs(speed4))));

//     if (maxSpeed > maxVel)
//     {
//         speed1 = speed1 * (maxVel / maxSpeed);
//         speed2 = speed2 * (maxVel / maxSpeed);
//         speed3 = speed3 * (maxVel / maxSpeed);
//         speed4 = speed4 * (maxVel / maxSpeed);
//     }

//     if (maxSpeed * power == 0)
//     {
//         angle1 = 45;
//         angle2 = -45;
//         angle3 = 45;
//         angle4 = -45;

//         rightFrontPID.reset();
//         leftFrontPID.reset();
//         leftBackPID.reset();
//         rightBackPID.reset();
//     }
//     // if(first) {
//     //     float magnetrn = serial_data.encoder_four;
//     //     while(magnetrn == serial_data.encoder_four) {
//     //         printf("Waiting for magnet to be set\n");
//     //         printf("magnetrn: %f\n", magnetrn);
//     //         magnetrn = serial_data.encoder_four;
//     //         pros::delay(10);
//     //     }
//     //     first = false;
//     // }
//     const double factor = 10.0;
//     rightFront.move(speed1 * factor, -angle1, power);
//     leftFront.move(speed2 * factor, -angle2, power);
//     leftBack.move(speed3 * factor, -angle3, power);
//     rightBack.move(speed4 * factor, -angle4, power);
// }

// void SwerveDrive::test(double speed, double angle, double power, int assembly)
// {
//     // 1 for right front, 2 for left front, 3 for left back, 4 rightback
//     if (assembly == 1)
//     {
//         rightFront.move(speed, angle, power);
//     }
//     else if (assembly == 2)
//     {
//         leftFront.move(speed, angle, power);
//     }
//     else if (assembly == 3)
//     {
//         leftBack.move(speed, angle, power);
//     }
//     else if (assembly == 4)
//     {
//         rightBack.move(speed, angle, power);
//     }
//     else
//     {
//         pros::lcd::print(6, "Help, thats not an assembly num");
//     }
// }

// void SwerveDrive::reset_position()
// {
//     rightFront.zero();
//     leftFront.zero();
//     leftBack.zero();
//     rightBack.zero();
// }