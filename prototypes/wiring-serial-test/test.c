#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <stdlib.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

int main ()
{
  char serialBuf[13];
  int serial_port;
  if ((serial_port = serialOpen ("/dev/ttyS0", 9600)) < 0) {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno));
    return 1;
  }

  while(1) {
    if (serialDataAvail(serial_port) > 0) {
      for (int i = 0; i < sizeof(serialBuf); i++) {
        serialBuf[i] = serialGetchar(serial_port);
      }
    serialFlush(serial_port);
  }

  printf("%s\n", serialBuf);


  }
  
   
  

  

}