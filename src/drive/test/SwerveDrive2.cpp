#include "drive/test/SwerveDrive2.h"

// Physical constants
#define TRACK_LENGTH 9.4f
#define TRACK_WIDTH 11.6f
#define MAX_SPEED_CONSTRAINT 1

// radius from center to each wheel
const double RADIUS = sqrt(pow(TRACK_LENGTH, 2) + pow(TRACK_WIDTH, 2));

//used to be const

const double rotKP = 2;
const double rotKI = 0;
const double rotKD = 6.9;

//front right
const double rotKP1 = rotKP + 0;
const double rotKI1 = rotKI + 0;
const double rotKD1 = rotKD - .12;
//front left
const double rotKP2 = rotKP + 0;
const double rotKI2 = rotKI + 0;
const double rotKD2 = rotKD - .05; 
//back left
const double rotKP3 = rotKP + 0;
const double rotKI3 = rotKI + 0;
const double rotKD3 = rotKD + 0; 
//back right
const double rotKP4 = rotKP + 0;
const double rotKI4 = rotKI + 0;
const double rotKD4 = rotKD + 0; 


int changeVar = 0;
// const double rotKD = 0.75;

lemlib::PID rightFrontPID(rotKP1, rotKI1, rotKD1);
lemlib::PID leftFrontPID(rotKP2, rotKI2, rotKD2);
lemlib::PID leftBackPID(rotKP3, rotKI3, rotKD3);
lemlib::PID rightBackPID(rotKP4, rotKI4, rotKD4);
float off = 0;
SwerveDrive2::SwerveDrive2()
    : rightFront(&rightFrontBottomMotor,
                 &rightFrontTopMotor,
                 &serial_data.encoder_four,
                 rightFrontPID,
                 off),
      leftFront(&leftFrontBottomMotor,
                &leftFrontTopMotor,
                &serial_data.encoder_one,
                leftFrontPID,
                off),
      leftBack(&leftBackBottomMotor,
                &leftBackTopMotor,
                &serial_data.encoder_three,
                leftBackPID,
                off),
      rightBack(&rightBackBottomMotor,
                &rightBackTopMotor,
                &serial_data.encoder_two,
                rightBackPID,
                off)
{ 
}

// anglewrapped [-180, 180], positive is clockwise, zero is straight ahead
// +x to the right, +y is downfield, power is scalar multiplier

void SwerveDrive2::move(double x, double y, double angle, double power)
{
    
    double angleModifer = 0;

    double speed = sqrt(pow(x, 2) + pow(y, 2));
    if(abs(x == 0)){
        x = .00001;
    }
    if(abs(y == 0)){
        y = .00001;
    }

    double vecAngle;
    if(abs(x) >.1 || abs(y) > .1){
        vecAngle = radToDeg(atan(y/x));
    } else{
        vecAngle = 0;
    }
    
    //has a hard switch
    if(vecAngle < -90 || vecAngle > 90){
        speed*=-1;
    }
    
    printf("%f\t%f\t%f\n", vecAngle, x, y);
    const double factor = 1.0;
    // rightFront.move(x * factor, -y * factor, power);
    // leftFront.move(x * factor, -y * factor, power);
    // leftBack.move(x * factor, -y * factor, power);
    // rightBack.move(x * factor, -y * factor, power);

    rightFront.move(speed, vecAngle, power);
    leftFront.move(speed, vecAngle, power);
    leftBack.move(-speed, vecAngle, power);
    rightBack.move(-speed, vecAngle, power);
}

void SwerveDrive2::reset_position()
{
    rightFront.zero();
    leftFront.zero();
    leftBack.zero();
    rightBack.zero();
}