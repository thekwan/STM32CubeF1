#include <iostream>
#include <fstream>
#include <stdlib.h>

#include "uart.h"
#include "map.h"
#include "ui.h"

#define MAX_BYTE_REPEAT_CNT    128

MapManager  mapmng;

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

// Only for reference
//typedef struct _distance_frame {
//    char speed_rpm[2];      // [3:2]
//    char start_angle[2];    // [5:4]
//    char dist_and_qual[24]; // [29:6]
//    char end_angle[2];      // [31:30]
//    char parity[2];         // [33:32]
//} dist_frame_t;

void FindLidarFrameStart(const char *device_file, const int baud_rate) {
    std::cout << "/**********************************" << std::endl;
    std::cout << " * UART TEST PROGRAM (Loop Test)  *" << std::endl;
    std::cout << " **********************************/" << std::endl;

    std::string rcv_string;

    UartDriverLite *uart0;

    if (baud_rate > 0) {
        uart0 = new UartDriverLite(device_file, 115200);
    }
    else {
        uart0 = new UartDriverLite(device_file);
    }

    u8 frameData[32];
    
    /* Find the frame sync '0x03', '0x08'
     */
    while(1) {
        /* MAIN #1: Find the frame sync bytes.
         */
        char byte, byte_prev;
        int  find_status = 0;
        bool find_sync = false;
        bool find_1st_byte = false;
        int  byte_repeat_cnt = 0;
        do {
            uart0->ReceiveByte(&byte);

            if (byte == byte_prev) {
                byte_repeat_cnt++;
            } else {
                byte_repeat_cnt = 0;
            }
            byte_prev = byte;
            
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
        } while (find_sync==false && byte_repeat_cnt < MAX_BYTE_REPEAT_CNT);

        if (byte_repeat_cnt >= MAX_BYTE_REPEAT_CNT) {
            std::cout << "[INFO] Can't find frame sync anymore. It will be terminated.\n";
            break;
        }
        
        //std::cout << "[INFO] Find the start sync of the frame.\n";

        /* MAIN #2: Get data & check the end bytes.
         */
        for (int i = 0; i < 32; i++) {
            uart0->ReceiveByte(&byte);
            frameData[i] = (u8) byte;
        }

        /* Main #3: Check the end bytes of the frame.
         */
        char ebyte0, ebyte1;
        uart0->ReceiveByte(&ebyte0);
        uart0->ReceiveByte(&ebyte1);
        if (!(ebyte0 == (char)0x55 && ebyte1 == (char)0xAA)) {
            //std::cout << "[0] = " << std::hex << (ebyte0 & 0xff) << "  ";
            //std::cout << "[1] = " << std::hex << (ebyte1 & 0xff) << "\n";
            std::cout << "[INFO] Invalid end byte of the frame.\n";
            continue;
        }

        //std::cout << "[INFO] Received the valid distance frame.\n";

        mapmng.push_frame_data((const u8 *)frameData);
    }
}

int main(int argc, char *argv[]) {
    //GenerateTestByteStream();
    //FindLidarFrameStart("/dev/ttyUSB0", 115200);
    FindLidarFrameStart("frame_sample.dat", -1);
    //DumpLidarFrame("/dev/ttyUSB0", "frame_sample.dat", int (34*1024));

    //mapmng.check_all_map_points();
    std::cout << "Found reliable points = " << mapmng.get_map_point_num() << std::endl;
    initOpenGL(argc, argv);

    return 0;
}
