#include "serial/serial_comm.h"
#include "Nanopb/pb_decode.h"
#include "msg.pb.h"
int nRead;

// Initialize data buffer

float otos_data[7];
float otos_offset[3];
vector<vector<int>> lens_data;

float otos_x;
float otos_y;
float otos_heading;

DataMessage serial_data = DataMessage_init_zero;

DataMessage data_msg = DataMessage_init_zero;

int32_t oldmillsec = 0;

int serialcounter = 0;

float encoder_one_avg[5] = {0, 0, 0, 0, 0};
float encoder_two_avg[5] = {0, 0, 0, 0, 0};
float encoder_three_avg[5] = {0, 0, 0, 0, 0};
float encoder_four_avg[5] = {0, 0, 0, 0, 0};

float encoder_one_avg_calcd;
float encoder_two_avg_calcd;
float encoder_three_avg_calcd;
float encoder_four_avg_calcd;

static FILE* serial_port = std::fopen("/dev/3", "r");

void print_buffer_serial(const char* buffer, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        printf("%02X", buffer[i]);
    }
    printf("\n");
}


bool readVarint32(FILE* fp, uint32_t &out) {
    out = 0;
    int shift = 0;
    while (true) {
        uint8_t byte;
        if (fread(&byte, 1, 1, fp) != 1) {
            // EOF or read error
            printf("Failed to read byte from file\n");
            return false;
        }
        out |= static_cast<uint32_t>(byte & 0x7F) << shift;
        if ((byte & 0x80) == 0) {
            // MSB not set → end of varint
            break;
        }
        shift += 7;
        if (shift >= 32) {
            // varint too long
            printf("Varint32 overflow\n");
            return false;
        }
    }
    return true;
}

// Reads one length-delimited Protobuf message from fp into 'msg'.
// Returns true on success, false on EOF or error.
bool ReadDelimitedFromFile(FILE* fp, DataMessage* msg) {
    // 1) Read length prefix
    uint32_t msg_size;
    if (!readVarint32(fp, msg_size)) {
        return false;
    }

    // printf("Reading message of size %u\n", msg_size);
    // 2) Read the raw message bytes
    uint8_t buffer[1000];
    if (msg_size > 0 && msg_size <= sizeof(buffer)) {
        size_t got = fread(buffer, 1, msg_size, fp);
        if (got != msg_size) {
            printf("Unexpected EOF when reading message (%zu of %u bytes)\n", got, msg_size);
            return false;
        }
    }
    pb_istream_t stream = pb_istream_from_buffer(buffer, msg_size);

    // print_buffer_serial((const char*)buffer, msg_size);
    
    // 3) Parse
    if (!pb_decode(&stream, DataMessage_fields, msg)) {
        printf("Failed to parse message of size %u\n", msg_size);
        if(msg_size == 37){
            //somehow got a bad size
            uint8_t byte2;
            fread(&byte2, 1, 1, fp);
            //trash a byte to cook it
            pros::delay(3);
            //wait a bit
        }
        printf("Error: %s\n", stream.errmsg);
        return false;
    }
    return true;
}



void serial_read(void *params)
{
    
    // printf("Serial read thread started\n");
    // Start serial on desired port
    // vexGenericSerialEnable(SERIALPORT - 1, 0);

    // Set BAUD rate
    // vexGenericSerialBaudrate(SERIALPORT - 1, VEX_BAUD_RATE);

    // Buffer to store serial data


    if(ReadDelimitedFromFile(serial_port, &data_msg)) {
        serialcounter++;
        
        if(!data_msg.tag) {

            printf("Tag is not set\n");
            return;
        }

        otos_data[0] = -data_msg.y;
        otos_data[1] = data_msg.x;
        otos_data[2] = data_msg.h;
        // otos_data[3] = data_msg.velocity.x;
        // otos_data[4] = data_msg.velocity.y;
        // otos_data[5] = data_msg.velocity.h;
        // otos_data[6] = data_msg.error;

        // printf("Position: %f %f %f\n", otos_data[0], otos_data[1], otos_data[2]);
        
        

        /*
        
        printf("Velocity: %f %f %f\n", otos_data[3], otos_data[4], otos_data[5]);
        printf("Error: %f\n", otos_data[6]);
        */
       encoder_one_avg[serialcounter % 5] = data_msg.encoder_one;
       encoder_two_avg[serialcounter % 5] = data_msg.encoder_two;
       encoder_three_avg[serialcounter % 5] = data_msg.encoder_three;
       encoder_four_avg[serialcounter % 5] = data_msg.encoder_four;
       
       if(serialcounter == 10){
       // printf("v5: %f\n", encoder_four_avg[4]);
       double encoder_one_total = 0;
       for(int i = 0; i < 5; i++) {
           encoder_one_total += encoder_one_avg[i];
       }
       encoder_one_avg_calcd = encoder_one_total / 5;
   
       double encoder_two_total = 0;
       for(int i = 0; i < 5; i++) {
           encoder_two_total += encoder_two_avg[i];
       }
       encoder_two_avg_calcd = encoder_two_total / 5;
   
       double encoder_three_total = 0;
       for(int i = 0; i < 5; i++) {
           encoder_three_total += encoder_three_avg[i];
       }
       encoder_three_avg_calcd = encoder_three_total / 5;
       
       double encoder_four_total = 0;
       for(int i = 0; i < 5; i++) {
           encoder_four_total += encoder_four_avg[i];
       }
       encoder_four_avg_calcd = encoder_four_total / 5;
       printf("calculated avgs: %f , %f , %f , %f\n", encoder_four_avg_calcd, encoder_three_avg_calcd, encoder_two_avg_calcd, encoder_one_avg_calcd);
       
   }
        // printf("Encoder 1: %f\n", data_msg.encoder_one);
        // printf("Encoder 2: %f\n", data_msg.encoder_two);
        // printf("Encoder 3: %f\n", data_msg.encoder_three);
        // printf("Encoder 4: %f\n", data_msg.encoder_four);
        
        otos_x = otos_data[0] + otos_offset[0];
        otos_y = otos_data[1] + otos_offset[1];
        otos_heading = otos_data[2] + otos_offset[2];
        serial_data = data_msg;
    } else{
        printf("Failed to decode message\n");
        // Print the error message
        serial_data = DataMessage_init_zero;
        
    } 
    
} 