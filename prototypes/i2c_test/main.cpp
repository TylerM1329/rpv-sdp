// Basic C++ Libraries
#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
 
// Libraries for TCP Configuration
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>

#include <pigpio.h>

#include <cstdlib>
#include <signal.h>

using namespace std;

void signal_callback_handler(int signum) {
   cout << "CTRL+C caught! Terminating..." << signum << endl;
   gpioTerminate();
   
   // Terminate program
   exit(signum);
}

void initialize_GPIO(void)
{
	if(gpioInitialise() < 0)
        printf("Failed to initialize pigpio\n");
    else
        printf("pigpio INIT OK\n");
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main() {
    initialize_GPIO();
    signal(SIGINT, signal_callback_handler);

    // STEERING TEST
    /*int handle = i2cOpen(1, 8, 0); // open I2C on Bus 1 targeting device 8
    while(1) {
        i2cWriteByte(handle, 50);
        usleep(200000);
        printf("Sent\n");
    }*/
    while(1) {
        int handle = i2cOpen(1, 3, 0);  // open target 6
        printf("Received %d", i2cReadByte(handle));
        usleep(200000);
    }
    


}

