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

#include <pigpiod_if2.h>

#include <cstdlib>
#include <signal.h>

using namespace std;

int pi = 0;

char data[2];


void signal_callback_handler(int signum) {
   cout << "CTRL+C caught! Terminating..." << signum << endl;
   pigpio_stop(pi);
   
   // Terminate program
   exit(signum);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main() {
    signal(SIGINT, signal_callback_handler);

    pi = pigpio_start(NULL, NULL);
    if (pi < 0)
        printf("Failed to connect to pigpio daemon. Attempted to connect to localhost:8888\n");
    else
        printf("pigpio INIT OK on localhost:8888\n");

    // STEERING TEST
    /*int handle = i2cOpen(1, 8, 0); // open I2C on Bus 1 targeting device 8
    while(1) {
        i2cWriteByte(handle, 50);
        usleep(200000);
        printf("Sent\n");
    }*/
    int handle = i2c_open(pi, 1, 3, 0);  // open target 3
    i2c_write_byte(pi, handle, 1);
    while(1) {
        printf("I2C handle: %d\n", handle);
        printf("Received %d bytes\n", i2c_read_device(pi, handle, data, 2)); //0~8
        printf("Data0: %d\n", data[0]);
        printf("Data1: %d\n", data[1]);
        //printf("Received %d\n", i2c_read_byte(pi, handle));
        //i2c_write_byte(pi, handle, 6);
        //usleep(200000);
        //i2c_write_byte(pi, handle, 7);
        usleep(60 * 1000); // 60 ms
}        /*for (int i = 0; i < 9; i++)
        {
            i2c_write_byte(pi, handle, i);
            usleep(1000000);
            printf("%d\n", i);
        }*/


}