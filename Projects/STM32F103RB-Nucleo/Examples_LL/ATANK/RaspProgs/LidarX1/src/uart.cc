#include <iostream>

//#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <string>
#include <thread>

#include "uart.h"

/* Baudrate 설정은 <asm/termbits.h>에 정의되어 있다.*/
/* <asm/termbits.h>는 <termios.h>에서 include된다. */
//#define BAUDRATE B115200
//#define BAUDRATE B38400
//#define BAUDRATE B9600
/* 여기의 포트 장치 파일을 바꾼다. COM1="/dev/ttyS1, COM2="/dev/ttyS2 */
//#define MODEMDEVICE "/dev/ttyS0"
//#define _POSIX_SOURCE 1 /* POSIX 호환 소스 */

//#define FALSE 0
//#define TRUE 1

//void __attribute__((weak)) UartMessageDisplayCallback(const char *message) {
//    fprintf(stdout, "%s", message);
//}

//#define _DEBUG_ENABLE_

UartDriverLite::UartDriverLite(const char *device)
   : _device(device), _baud_rate(-1), _open_success(false) {
    uart_fd = open(_device, O_RDWR | O_NOCTTY );
    if (uart_fd <0) {
        return;
    }

    _open_success = true;
}

UartDriverLite::UartDriverLite(const char *device, int baud_rate)
   : _device(device), _baud_rate(baud_rate), _open_success(false)
{
    //int fd,c, res;
    char buf[255];

    /* 읽기/쓰기 모드로 모뎀 장치를 연다.(O_RDWR)
     데이터 전송 시에 <CTRL>-C 문자가 오면 프로그램이 종료되지 않도록
     하기 위해 controlling tty가 안되도록 한다.(O_NOCTTY)
    */
    //fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
    //if (fd <0) {perror(MODEMDEVICE); exit(-1); }
    uart_fd = open(_device, O_RDWR | O_NOCTTY );
    if (uart_fd <0) {
        //perror(device_file);
        //UartMessageDisplayCallback("[ERROR] Can't open the device file.\n");
        return;
    }

    tcgetattr(uart_fd, &oldtio); /* save current serial port settings */
    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    /*
    BAUDRATE: 전송 속도. cfsetispeed() 및 cfsetospeed() 함수로도 세팅 가능
    CRTSCTS : 하드웨어 흐름 제어. (시리얼 케이블이 모든 핀에 연결되어 있는
              경우만 사용하도록 한다. Serial-HOWTO의 7장을 참조할 것.)
    CS8     : 8N1 (8bit, no parity, 1 stopbit)
    CLOCAL  : Local connection. 모뎀 제어를 하지 않는다.
    CREAD   : 문자 수신을 가능하게 한다.
    */
    //newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    if (baud_rate == 115200) {
        newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    }
    else if (baud_rate == 38400) {
        newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
    }
    else {
        newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    }

    /*
    IGNPAR   : Parity 에러가 있는 문자 바이트를 무시한다.
    ICRNL    : CR 문자를 NL 문자로 변환 처리한다. (이 설정을 안하면 다른
              컴퓨터는 CR 문자를 한 줄의 종료문자로 인식하지 않을 수 있다.)
    otherwise make device raw (no other input processing)
    */
    newtio.c_iflag = IGNPAR | ICRNL;

    /*
    Raw output.
    */
    newtio.c_oflag = 0;

    /*
    ICANON   : canonical 입력을 가능하게 한다.
    disable all echo functionality, and don't send signals to calling program
    */
#if defined(CANONICAL_MODE)
    newtio.c_lflag = ICANON;
#else
    newtio.c_lflag &= ~(ICANON);
    newtio.c_lflag &= ~(ECHO | ECHOE);
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 10;
#endif

    /*
    모든 제어 문자들을 초기화한다.
    디폴트 값은 <termios.h> 헤어 파일에서 찾을 수 있다. 여기 comment에도
    추가로 달아놓았다.
    */
    newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
    newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
    newtio.c_cc[VERASE]   = 0;     /* del */
    newtio.c_cc[VKILL]    = 0;     /* @ */
    newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
    newtio.c_cc[VSWTC]    = 0;     /* '\0' */
    newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
    newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    newtio.c_cc[VEOL]     = 0;     /* '\0' */
    newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    newtio.c_cc[VEOL2]    = 0;     /* '\0' */

    /*
    이제 modem 라인을 초기화하고 포트 세팅을 마친다.
    */
    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd,TCSANOW,&newtio);

    /*
     * Create thread for rx/tx transmission.
     */
    int thr_id;
    //int status;

    //UartMessageDisplayCallback("[INFO] UART open is success.\n");

#if 0   // TODO: tx thread is blocked. [temp]
#if 0
    thr_id = pthread_create(&p_thread[0], NULL, tx_thread, (void*) &fd);
    if(thr_id < 0) {
        perror("thread create error: tx_thread\n");
        return -1;
    }
#else
    thr_id = pthread_create(&p_thread[0], NULL, tank_control_thread, (void*) &uart_fd);
    if(thr_id < 0) {
        perror("thread create error: tank_control_thread\n");
        return -1;
    }
#endif
#endif

    //thr_id = pthread_create(&p_thread[1], NULL, rx_thread, (void*) &uart_fd, UartMessageDisplayCallback);
    //if(thr_id < 0) {
    //    UartMessageDisplayCallback("thread create error: rx_thread\n");
    //    return -1;
    //}

    //p_thread[1] = std::thread(rx_thread_wrapper, this, UartMessageDisplayCallback);

    //pthread_join(p_thread[0], (void**) &status);
    //pthread_join(p_thread[1], (void**) &status);

    //printf("Program End.. Bye~\n");

    //
    ///* restore the old port settings */
    //tcsetattr(uart_fd,TCSANOW,&oldtio);

    //SendMessageUart("reset");

    _open_success = true;

    return;
}

UartDriverLite::~UartDriverLite(void) {
    if (_baud_rate > 0) {
        /* restore the old port settings */
        tcsetattr(uart_fd,TCSANOW,&oldtio);
    }
    close(uart_fd);
}

void UartDriverLite::SendMessageUart(std::string message) {
    if (!_open_success) {
        std::cerr << "[ERROR] UART terminal is not opened!" << std::endl;
        return;
    }
    
    write(uart_fd, message.c_str(), message.size());
}

void UartDriverLite::ReceiveMessageUart(std::string &message) {
    char buf[1024];

    if (!_open_success) {
        std::cerr << "[ERROR] UART terminal is not opened!" << std::endl;
        return;
    }
    
    int res = read(uart_fd, buf, 1024);

    message = std::string(buf);
}

void UartDriverLite::SendByte(const char *data) {
    if (!_open_success) {
        std::cerr << "[ERROR] UART terminal is not opened!" << std::endl;
        return;
    }
    
    write(uart_fd, data, sizeof(char));
}

void UartDriverLite::ReceiveByte(char *data) {
    if (!_open_success) {
        std::cerr << "[ERROR] UART terminal is not opened!" << std::endl;
        return;
    }

    int res = read(uart_fd, data, sizeof(char));

#if defined(_DEBUG_ENABLE_)
    std::cout << "RX Byte[0x" << std::hex << (*data & 0xFF) << "]\n";
#endif
}
