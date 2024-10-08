#include <iostream>
#include <string>
#include <string.h>
#include <sstream>

#include <pigpiod_if2.h>

#include <cstdlib>
#include <signal.h>

using namespace std;
int pi;
int spi;

int initialize_GPIO(void)
{
	int pigpiod_handle = pigpio_start(NULL, NULL);
	if(pigpiod_handle < 0)
        printf("Failed to connect to pigpio daemon. Attempted to connect to localhost:8888\n");
    else
        printf("pigpio INIT OK on localhost:8888\n");
	return pigpiod_handle;
}

void signal_callback_handler(int signum) {
   cout << "CTRL+C caught! Terminating..." << signum << endl;
   //serClose(serialPort);
   spi_close(pi, spi);
   pigpio_stop(pi);

   
   // Terminate program
   exit(signum);
}



int main() {
    pi = initialize_GPIO();
    signal(SIGINT, signal_callback_handler);	// make sure to call this after calling gpioInitialise()
    spi = spi_open(pi, 0, 1000000, 0);          // baud rate affects data!
    char tx[3] = {1, 128, 0};                   // CH0: 128, CH1: 144
    char rx[3];
    printf("PIGPIOD Handle: %d\n", pi);
    printf("SPI Handle: %d\n", spi);
    while(1) {
        printf("TX: %d bytes\n", spi_xfer(pi, spi, tx, rx, 3));
        int data = (rx[1] << 8) | rx[2] & 0x3FF;
        printf("RX: %d\n", data);
        usleep(100000);
    }


}