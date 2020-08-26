#include <iostream>

#include "uart.h"

int main(void) {
    std::cout << "/**********************************" << std::endl;
    std::cout << " * UART TEST PROGRAM              *" << std::endl;
    std::cout << " **********************************/" << std::endl;

    std::string rcv_string;

    UartDriverLite uart0("/dev/ttyUSB0", 115200);

    uart0.SendMessageUart("TEST send string.\n");
    uart0.ReceiveMessageUart(rcv_string);

    std::cout << "Received data is => " << std::endl;
    std::cout << "'" << rcv_string << "'" << std::endl;

    return 0;
}
