#include <iostream>
#include <fstream>

#include "uart.h"

void uart_loop_test(void) {
    std::cout << "/**********************************" << std::endl;
    std::cout << " * UART TEST PROGRAM (Loop Test)  *" << std::endl;
    std::cout << " **********************************/" << std::endl;

    std::string rcv_string;

    UartDriverLite uart0("/dev/ttyUSB0", 115200);

    uart0.SendMessageUart("TEST send string.\n");
    uart0.ReceiveMessageUart(rcv_string);

    std::cout << "Received data is => " << std::endl;
    std::cout << "'" << rcv_string << "'" << std::endl;

    return;
}

typedef struct _distance_frame {
    char speed_rpm[2];      // [3:2]
    char start_angle[2];    // [5:4]
    char dist_and_qual[24]; // [29:6]
    char end_angle[2];      // [31:30]
    char parity[2];         // [33:32]
} DistFrame;

void GenerateTestByteStream(void) {
    std::ofstream ofs;
    ofs.open("byte_stream_sim.dat", std::ios::binary | std::ios::out);

    if (ofs.is_open() == false) {
        std::cout << "[ERROR] Can't open the file 'byte_stream_sim.dat'\n";
        return;
    }

    const unsigned char dummy[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    const unsigned char frame[34] = {
        0x03, 0x08,         // frame start sync
        0xff,               // speed INFO
        0x00,               // unknown byte_stream_sim
        0x01, 0x02,         // start angle
        0x01, 0x02, 0x03,   // distance (L,H), quality, ... repeat 8 times.
        0x01, 0x02, 0x03,
        0x01, 0x02, 0x03,
        0x01, 0x02, 0x03,
        0x01, 0x02, 0x03,
        0x01, 0x02, 0x03,
        0x01, 0x02, 0x03,
        0x01, 0x02, 0x03,
        0x0a, 0x0b,         // end angle
        0x55, 0xAA          // frame end sync
    };

    ofs.write((const char *)dummy, 8);
    ofs.write((const char *)frame, 34);
    ofs.write((const char *)dummy, 8);
    ofs.write((const char *)dummy, 8);
    ofs.write((const char *)dummy, 8);
    ofs.write((const char *)dummy, 8);

    ofs.close();
}

void DumpLidarFrame(const char *device_file, const char *dump_file, int max_size) {
    std::ofstream dump_fs;
    dump_fs.open(dump_file, std::ios::binary | std::ios::out);

    UartDriverLite uart0(device_file, 115200);

    char byte;
    int count = 0;
    while(count++ < max_size) {
        uart0.ReceiveByte(&byte);
        dump_fs.write((const char *)&byte, 1);
    }

    dump_fs.close();
}

typedef unsigned short u16;
typedef unsigned char  u8;

void CheckLidarFrame(const char *fdata) {
    DistFrame  dframe;

    u16 speed_rpm;
    u16 start_angle, end_angle;
    u16 distance[8];
    u8  quality[8];
    
    speed_rpm   = ((u16)fdata[1]) << 8 | (u16)fdata[0];
    start_angle = ((u16)fdata[3]) << 8 | (u16)fdata[2];
    start_angle = start_angle/64 - 640;

    for(int i = 0; i < 8; i++) {
        distance[i] = ((u16)fdata[5+3*i]) << 8 | (u16)fdata[4+3*i];
        quality[i]  = (u8)fdata[6+3*i];
    }

    end_angle = ((u16)fdata[29]) << 8 | (u16)fdata[28];
    end_angle = end_angle/64 - 640;

    // no parity check
    
    std::cout << "speed_rpm   = " << speed_rpm << std::endl;
    std::cout << "start_angle = " << start_angle << std::endl;
    for(int i = 0; i < 8; i++) {
        std::cout << "distance[" << i << "] = " << distance[i];
        std::cout << "\tquality = " << (u16)quality[i] << std::endl;
    }
    std::cout << "end_angle   = " << end_angle << std::endl;
    std::cout << std::endl;
}

void FindLidarFrameStart(const char *device_file) {
    std::cout << "/**********************************" << std::endl;
    std::cout << " * UART TEST PROGRAM (Loop Test)  *" << std::endl;
    std::cout << " **********************************/" << std::endl;

    std::string rcv_string;

    UartDriverLite uart0(device_file, 115200);

    char frameData[32];
    
    /* Find the frame sync '0x03', '0x08'
     */
    while(1) {
        /* MAIN #1: Find the frame sync bytes.
         */
        char byte;
        int  find_status = 0;
        bool find_sync = false;
        bool find_1st_byte = false;
        do {
            uart0.ReceiveByte(&byte);
            
            if (find_1st_byte) {
                find_1st_byte = false;
                if (byte == 0x08) {
                    find_sync = true;
                }
            }
            else {
                if (byte == 0x03) {
                    find_1st_byte = true;
                }
            }
        } while (find_sync==false);
        
        std::cout << "[INFO] Find the start sync of the frame.\n";

        /* MAIN #2: Get data & check the end bytes.
         */
        for (int i = 0; i < 32; i++) {
            uart0.ReceiveByte(&byte);
            frameData[i] = byte;
        }

        /* Main #3: Check the end bytes of the frame.
         */
        char ebyte0, ebyte1;
        uart0.ReceiveByte(&ebyte0);
        uart0.ReceiveByte(&ebyte1);
        if (!(ebyte0 == (char)0x55 && ebyte1 == (char)0xAA)) {
            std::cout << "[0] = " << std::hex << (ebyte0 & 0xff) << "  ";
            std::cout << "[1] = " << std::hex << (ebyte1 & 0xff) << "\n";
            std::cout << "[INFO] Invalid end byte of the frame.\n";
            continue;
        }

        std::cout << "[INFO] Received the valid distance frame.\n";

        CheckLidarFrame((const char *)frameData);
    }
}

int main(void) {
    //GenerateTestByteStream();
    FindLidarFrameStart("/dev/ttyUSB0");
    //DumpLidarFrame("/dev/ttyUSB0", "frame_sample.dat", int (34*1024));

    return 0;
}
