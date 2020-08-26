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

//void __attribute__((weak)) UartMessageDisplayCallback(const char *message);

class UartDriverLite {
public:
    UartDriverLite(const char *device, int baud_rate);
    ~UartDriverLite();

    //int  OpenChannelUart(void);
    //void CloseChannelUart(void);
    void SendMessageUart(std::string message);
    void ReceiveMessageUart(std::string &message);

private:
    const char *_device;
    int _baud_rate;
    bool _open_success;
    //const char *device_file;
    int uart_fd;
    //pthread_t  p_thread[2];
    std::thread  p_thread[2];
    struct termios oldtio, newtio;
};
