#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS
#include "api.h"
#include "init/devices.h"
#include <vector>
#include "msg.pb.h"

using namespace std;

// The bits per second that will be sent on the serial bus
#define VEX_BAUD_RATE 115200U

// The buffer size
#define BUFFER_SIZE (uint16_t)400

// Function that updates data buffer from reading the serial bus
void serial_read(void *params);

// Global data buffer
extern std::string max485_data;

extern float otos_data[7];
extern float otos_offset[3];

extern DataMessage serial_data;


extern float otos_x;
extern float otos_y;
extern float otos_heading;


extern float encoder_one_avg[5];
extern float encoder_two_avg[5];
extern float encoder_three_avg[5];
extern float encoder_four_avg[5];

extern float encoder_one_avg_calcd;
extern float encoder_two_avg_calcd;
extern float encoder_three_avg_calcd;
extern float encoder_four_avg_calcd;

extern vector<vector<int>> lens_data;

extern int serialcounter;

extern int nRead;

//  Currently in Use
// [0] -> pos_x
// [1] -> pos_y
// [2] -> pos_h
// [3] -> vel_x
// [4] -> vel_y
// [5] -> vel_h
// [6] -> error

// Not in Use
// [0] -> pos_x
// [1] -> pos_y
// [2] -> pos_h
// [3] -> vel_x
// [4] -> vel_y
// [5] -> vel_h
// [6] -> accel_x
// [7] -> accel_y
// [8] -> accel_h
// [9] -> error
#endif