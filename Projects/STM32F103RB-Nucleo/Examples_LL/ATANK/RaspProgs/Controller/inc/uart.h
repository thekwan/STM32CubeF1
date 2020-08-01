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

void __attribute__((weak)) UartMessageDisplayCallback(const char *message);

class UartDriverLite {
public:
    UartDriverLite(const char *device) : device_file(device) {}
    ~UartDriverLite() {}

    void OpenChannelUart(void);
    void CloseChannelUart(void);
    void SendMessageUart(std::string message);
    void ReceiveMessageUart(std::string *message) {};
    void rx_thread(void (*print_func)(const char*));

    static void rx_thread_wrapper(UartDriverLite *handle, void (*callback)(const char*)) {
        handle->rx_thread(callback);
    }
private:
    const char *device_file;
    int uart_fd;
    //pthread_t  p_thread[2];
    std::thread  p_thread[2];
    struct termios oldtio, newtio;
};
