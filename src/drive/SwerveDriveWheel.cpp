// #include "drive/SwerveDriveWheel.h"

// // in deg
// #define ANGLE_MARGIN_OF_ERROR 0.25
// #define IDLE_ANGLE_MARGIN_OF_ERROR 8
// // max velocity of motors, in RPM ticks
// #define MAX_MOTOR_SPEED_TICKS 3

// SwerveDriveWheel::SwerveDriveWheel(pros::Motor *motorTop, pros::Motor *motorBot, float *rotateEncoder, lemlib::PID &pid,
//                                    float offset)
//     : motorTop(motorTop), motorBot(motorBot),

//       // float with angle in it from serial
//       rotateEncoder(rotateEncoder), PIDr(pid)
// {
//     // made rot for a minimal amount more clarity
//     current_r = getMagnetAngle();
//     // rename
//     currentMagnet = 0;

//     // zero(offset);

//     // zeros motor positions
//     int stat1 = motorTop->tare_position();
//     int stat2 = motorBot->tare_position();

//     // status of motors, example error
//     pros::lcd::print(7, "%i : %i", stat1, stat2);
// }

// float SwerveDriveWheel::getMagnetAngle()
// {
//     // the difference between where the wheel is and where the zero was set
//     float zeroed = *rotateEncoder - zeroOffset;
//     if (zeroed < 0)
//     {
//         zeroed += 1;
//     }
//     else if (zeroed > 1)
//     {
//         zeroed -= 1;
//     }
//     // measured in centidegrees [0, 360]
//     return (((1 - zeroed) * 360));
// }

// float SwerveDriveWheel::getAngle()
// {
//     // get the current angle of the wheel in degrees
//     float angleTop = motorTop->get_position();
//     float angleBot = motorBot->get_position();

//     // get the average of the two angles (this is where the swerve wheel is
//     // facing)
//     float avgAngle = (angleTop + angleBot) / 2.0f;

//     // 0 180 or 360 and check valid data first
//     if ((int)currentMagnet % 180 == 0 && serial_data.tag)
//     {
//         float newMagnet = fmod(getMagnetAngle() + (avgAngle * gearratio),
//                                360.0f); // set the latest magnet to the current angle

//         if (currentMagnet < 0)
//         {
//             currentMagnet += 360; // wrap around to positive
//         }
//     }
//     float totalAngle = -avgAngle * gearratio + currentMagnet; // convert to degrees
//     float finalAngle = fmod(totalAngle, 360.0f);

//     if (finalAngle < 0)
//     {
//         finalAngle += 360; // wrap around to positive
//     }

//     bool debug = true;
//     if (debug)
//     {
//         pros::lcd::print(0, "AngleTop: %f\n", angleTop);
//         pros::lcd::print(1, "AngleBot: %f\n", angleBot);
//         pros::lcd::print(2, "AvgAngle: %f\n", avgAngle);
//         // pros::lcd::print(3, "TotalAngle: %f\n", totalAngle);
//         pros::lcd::print(3, "getAngle(): %f\n", finalAngle);
//     }
//     return finalAngle;
// }

// /** Zeroes the wheel by updating the offset, and returns the new offset */
// void SwerveDriveWheel::zero(float offset)
// {
//     if (offset == -1)
//     {
//         offset = getAngle();
//     }
//     // set the zeroOffset to the current angle
//     zeroOffset = offset;
// }

// double SwerveDriveWheel::calculatePID(double target, double current, bool idle)
// {
//     double error = calcAngleDiff(degreesToRadians(current), degreesToRadians(target));

//     error = radiansToDegrees(error); // Convert back to degrees for PID math

//     // // printf("Err: %f\n Curr: %f\n", error, current);

//     if (idle)
//     {
//         if (abs(error) < IDLE_ANGLE_MARGIN_OF_ERROR)
//         {
//             return 0;
//         }
//     }
//     else
//     {
//         if (abs(error) < ANGLE_MARGIN_OF_ERROR)
//         {
//             return 0;
//         }
//     }

//     // Calculate PID output
//     double output_power = PIDr.update(error);

//     if (output_power > 127)
//     {
//         output_power = 127;
//     }
//     else if (output_power < -127)
//     {
//         output_power = -127;
//     }

//     // Scale output to reasonable motor power
//     return output_power;
// }

// void SwerveDriveWheel::move(double speed, double angle, double power)
// {
//     target_r = angle;

//     // Both cases are taken to see whats a shorter path
//     // the opposite angle case, to figure out if it's distance is shorter
//     double opp_angle = target_r - 180;

//     // reference angle
//     float retVal = getAngle();
//     bool reverse = true;

//     double angleFromTarget = calcAngleDiff(degreesToRadians(retVal), degreesToRadians(angle));
//     double oppAngleFromTarget = calcAngleDiff(degreesToRadians(retVal), degreesToRadians(opp_angle));

//     // set new variable values if the oppAngle is a shorter distance
//     if (abs(oppAngleFromTarget) < abs(angleFromTarget))
//     {
//         angleFromTarget = oppAngleFromTarget;
//         target_r = opp_angle;
//         reverse = !reverse;
//     }

//     if (reverse)
//     {
//         speed *= -1.0;
//     }

//     if (retVal == current_r)
//     {
//         // printf("same val \n");
//         // interpolate if not 0s
//         if (lastValids[0] != 0 && lastValids[1] != 0)
//         {
//             retVal = lastValids[1] + (lastValids[1] - lastValids[0]);
//             //* (reverse?1:-1);
//             // printf("abc %f %f %f\n",lastValids[0],lastValids[1],retVal);
//         }
//     }
//     else
//     {
//         // fill lastReads
//         lastValids[0] = lastValids[1];
//         lastValids[1] = retVal;
//     }

//     // lastRead = retVal;

//     current_r = retVal;

//     speed *= cos(angleFromTarget);

//     double rPower = calculatePID(target_r, current_r, speed == 0);

//     // turns on prints for this method
//     bool debug = false;
//     if (debug)
//     {
//         pros::lcd::print(1, "target_r: %f\n", target_r);
//         pros::lcd::print(2, "current_r: %f\n", current_r);
//         pros::lcd::print(3, "angleFromTarget: %f\n", angleFromTarget);
//         pros::lcd::print(4, "encoder: %f\n", serial_data.encoder_four);
//         pros::lcd::print(5, "rPower: %f\n", rPower);
//         pros::lcd::print(6, "speed %f\n", speed);
//     }

//     /*
//         ALL THE BELOW CODE IS TO OPTIMIZE
//         MOTOR POWER USAGE, AND HANDLE EDGE CASES

//         abs(speed) > 2, meant to make it so that there's no rapid rotation,
//         when the input is 0, causing the angle to be anywhere

//         The code segment with sum1, sum2, and delta meant to make the full use
//         of the motor power while staying in range [-127, 127], prioritizing
//        rotation

//         delta = the overflow of the max mumber above range, which when subtracted
//         from either motor power sums, EFFECTIVELY only subtracts from the
//         linear speed, keeping rotational power

//         CURRENT SITUATION:
//         this delta setup is more unstable than without it, so patch up some
//        instabilities next time

//         final configuration:
//         SwerveDrive.cpp
//             //set multiplers to 127
//             rightFront.move(speed1*127.0, angleTop, power);

//         right now it is at 75 as it's more stable that way for now

//     */

//     double maxPower = abs(speed) + abs(rPower);

//     // if (maxPower > 2) {

//     // original output for each motors, will scale them accordingly below
//     double sum1 = -speed + rPower;
//     double sum2 = speed + rPower;

//     if (maxPower > 127)
//     {
//         sum1 /= maxPower / 127;
//         sum2 /= maxPower / 127;
//     }

//     // if (abs(sum1) > 0 && abs(sum1) < 10) sum1 *= 10/sum1;
//     // if (abs(sum2) > 0 && abs(sum2) < 10) sum2 *= 10/sum2;

//     motorTop->move_velocity(sum1 * 600 / 127);
//     motorBot->move_velocity(sum2 * 600 / 127);

//     // printf("sum1: %f\n", sum1);
//     // printf("sum2: %f\n", sum2);
//     // delta = the overflow of the max mumber above range, which when subtracted
//     // from either motor power sums, EFFECTIVELY only subtracts from the
//     // linear speed, keeping rotational power

//     // double delta = 0.0;
//     // if (abs(sum1) > abs(sum2)) {
//     //     // SUM ONE IS LARGER
//     //     if (sum1 > 127.0)
//     //     {
//     //         delta = sum1 - 127;
//     //     }
//     //     else if (sum1 < -127)
//     //     {
//     //         delta = sum1 + 127;
//     //     }
//     //     motor1->move(sum1 - delta);
//     //     motor2->move(sum2 + delta);
//     // } else {
//     //     // SUM TWO IS LARGER
//     //     if (sum2 > 127)
//     //     {
//     //         delta = sum2 - 127;
//     //     }
//     //     else if (sum2 < -127)
//     //     {
//     //         delta = sum2 + 127;
//     //     }
//     //     motor1->move(sum1 + delta);
//     //     motor2->move(sum2 - delta);
//     // }

//     //} else {
//     // when input power is 0, to eliminate rapid angle changes
//     //    motor1->move(0);
//     //    motor2->move(0);
//     //}
// }