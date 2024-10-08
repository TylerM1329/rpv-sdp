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
#define PORT 54000

#include <cstdlib>
#include <signal.h>

using namespace std;

int pi = 0;

int handle = 0;
char data[4];
int status, valread, client_fd;
char dataOut[21];

void signal_callback_handler(int signum) {
   cout << "CTRL+C caught! Terminating..." << signum << endl;

   i2c_close(handle, pi);
   pigpio_stop(pi);
   close (client_fd);
   
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

    handle = i2c_open(pi, 1, 3, 0);  // open target 3
    static char usSensors[4];

    
    struct sockaddr_in serv_addr;
    char* hello = "Hello from client";
    char buffer[1024] = { 0 };
    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }
 
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
 
    // Convert IPv4 and IPv6 addresses from text to binary
    // form
    if (inet_pton(AF_INET, "192.168.5.135", &serv_addr.sin_addr)
        <= 0) {
        printf(
            "\nInvalid address/ Address not supported \n");
        return -1;
    }
 
    if ((status
         = connect(client_fd, (struct sockaddr*)&serv_addr,
                   sizeof(serv_addr)))
        < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }

    send(client_fd, hello, strlen(hello), 0);

    while(1) {
        i2c_read_device(pi, handle, usSensors, 4);
        snprintf(dataOut, 21, "F%d-B%d|", usSensors[0], usSensors[1]);
        printf("%s", dataOut);
        send(client_fd, dataOut, strlen(dataOut), MSG_DONTWAIT);
        // printf("Front: %dcm\n", usSensors[0]);
	    // printf("Back: %dcm\n", usSensors[1]);
	    // printf("Left: %dcm\n", usSensors[2]);
	    // printf("Right: %dcm\n", usSensors[3]);
        usleep(50*1000);
    }
}


