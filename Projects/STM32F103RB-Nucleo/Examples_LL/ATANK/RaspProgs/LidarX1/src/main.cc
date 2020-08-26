#include <iostream>

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

int main(void) {
    return 0;
}
