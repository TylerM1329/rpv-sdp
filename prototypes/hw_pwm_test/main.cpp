#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include <iostream>

using namespace std;

int serialPort;
char serialBuf[13];

void signal_callback_handler(int signum) {
   cout << "CTRL+C caught! Terminating..." << signum << endl;
   gpioTerminate();
   serClose(serialPort);
   // Terminate program
   exit(signum);
}

int main() {
    if(gpioInitialise() < 0)
        printf("Failed to initialize pigpio\n");
    else
        printf("pigpio INIT OK\n");
    signal(SIGINT, signal_callback_handler);
    if ((serialPort = serOpen("/dev/ttyS0", 115200, 0)) < 0)
				cout << "Failed to open serial port!" << endl;
			else
				cout << "Listening for serial commands at /dev/ttyS0 at 9600 baud" << endl;
    //gpioSetMode(18, PI_OUTPUT);
    //gpioServo(18, 2000);
    while(1) {
        if (serDataAvailable(serialPort) > 0) {
            printf("Read %d bytes\n", serRead(serialPort, serialBuf, 13));
        }
        printf("Serial RX: %s\n", serialBuf);
        usleep(15000);
    }

    
    //usleep(5e6);
}