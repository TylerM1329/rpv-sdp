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
char serialBuf[18];
char preParseBuf[18];
int serialHdl = 0;
int serialRXbyteCount = 0;

void signal_callback_handler(int signum) {
   cout << "CTRL+C caught! Terminating..." << signum << endl;
   pigpio_stop(pi);
   serial_close(pi, serialHdl);
   
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

    char port[] = "/dev/ttyS0";
	serialHdl = serial_open(pi, port, 115200, 0);

    while(1) {
        int serialAvail = serial_data_available(pi, serialHdl);
        printf("Serial available: %d\n", serialAvail);
		if (serialAvail >= 17)  
            // used >= 13 here because sometimes serialAvail is 2, 11, whatever so 13 is to filter out bad buffer counts
            // works with this, no glitching at output buffer with 100ms transmit delay
			serialRXbyteCount = serial_read(pi, serialHdl, serialBuf, sizeof(serialBuf));
		printf("RX serialBuf: %s\n", serialBuf);

        if (serialRXbyteCount == 17) {
            for (int i = 0; i < 17; i++) {
                preParseBuf[i] = serialBuf[i];
            }
        }
        printf("RX preParseBuf: %s\n", preParseBuf);

        usleep(50*1000);
    }       
}