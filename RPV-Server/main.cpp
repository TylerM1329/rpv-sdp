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
#include "functions.h"

using namespace std;
void signal_callback_handler(int signum);

const int camera_servo = 18;
const int brake_servo = 13;				
const int ultrasonicAddr = 3;			// I2C address for ultrasonic subsystem
const int lightingAddr = 6;				// I2C address for lighting subsystem
// using addresses below 10 can cause issues
const int lidarAddr = 0x20;				// I2C address for lidar subsystem
const int GPSAddr = 0x15;				// I2C address for GPS Subsytem (Named Telementary)

int pi = 0;								// var to hold pigpiod handle
int ultrasonicHdl = 0;					// var to hold ultrasonic subsystem handle
int lightingHdl = 0;					// var to hold lighting subsystem handle
int adcHdl = 0;							// var to hold SPI ADC handle
int serialHdl = 0;						// var to hold serial handle
int clientSocket = 0;					// var to hold TCP client socket number
int lidarHdl = 0;						// var to hold lidar sensor subsystem handle
int GPSHdl = 0;							// var to hold GPS Sensor (Telementary) subsystem handle

const int brakeRelaxed = 2050;			// var to hold servo position that relaxes brake
const int brakeFull = 2250;				// var to hold servo position that brakes fully

double destination_data[6][3] = {
	// 2D array to hold GPS datapoints [longitude, latitude, course angle]
	{34.11463547, -117.70419312},
	{34.11457825, -117.70418549,0}, // point 1
	{34.11465073,-117.70417786,0}, 	// point 2
	{34.11465454,-117.70425415,0}, 	// point 3
	{-1,-1,-1}		// point end (must be [-1,-1,-1])
};

int gps_destination_counter = 0;

int main()
{
	char ipAddr[15];
	int port = 0;
	int serialRXenable = 0;
	bool CVenable = 0;							// var to hold state of (computer vision) lane keeping option.
	int ASenable = 0;							// var to hold state of active safety option.
	int TCPRXbyteCount = 0;						// byte count of received message
	int serialRXbyteCount = 0;
	int const serialMaxBytes = 17;				// max bytes of string for processing in bus
	char socketBuf[512];						// receive buffer for TCP socket
	char preParseBuf[serialMaxBytes + 1];		// buffer that enters parsing logic
	char serialBuf[serialMaxBytes + 1];			// receive buffer for serial port
	int serialAvail = 0;						// var to hold available bytes to be read
	int steerValue = 0;							// var to hold parsed steering value
	int accelValue = 0;							// var to hold parsed acceleration value
	int controlledAccelValue = 0;				//
	int brakeValue = 0;							// var to hold parsed braking value
	int controlledBrakeValue = 0;				//
	int buttons1 = 0;							// var to hold first set of buttons (X and B). used for camera panning. these get held down
	int buttons2 = 0;							// var to hold left and right buttons. used for turn signals. brake/reverse handled internally. future: headlight control. CV toggle
	int gear = 0;								// flag to hold gear. 0 = fwd, 1 = rvs.
	int lidarDist = 0;
	int GPSLoca   = 0;							// Var to hold GPS data
	char spi_tx[3] = {1, 128, 0};  				// command to read CH0 from MCP3008
	char spi_rx[3];								// array to hold received SPI data
	int adc_data = 0;							// var to hold adc value
	int deadband = 11;							// var to hold deadband value for steering logic

	bool dataReady = 0;							// flag used to tell parsing logic if data coming in is good
	char usSensors[4];							// array to hold received ultrasonic sensor distance data
												// 4 bytes (1 byte for each sensor)
	bool haltFlag = 0;
	char AS_state = 0;							// current state of ASM. 
	int AS_timer = 0;
	
	if (!init_IO(pi, ultrasonicHdl, ultrasonicAddr, lightingHdl, lightingAddr, adcHdl, serialHdl, lidarHdl, lidarAddr, GPSHdl, GPSAddr))
		printf("[!!] One or more I/O devices failed to initialize!\n");
	signal(SIGINT, signal_callback_handler);	// make sure to call this after calling init_IO()

	// Disable motors and relax brake servo at program start
	disable_motors(pi);
   	set_servo_pulsewidth(pi, brake_servo, brakeRelaxed);	// relax brake servo
	run_brake_lights(0, pi, lightingHdl);					// turn off brake lights

	get_network_options(ipAddr, port);
	printf("Accepted values: IP = %s Port = %d", ipAddr, port);
	get_user_options(serialRXenable, ASenable);
	cout << "Opening Socket! Listening at " << ipAddr << ":" << port << endl;

	loop:
	// Call TCP server setup function and store socket number
	clientSocket = setup_TCP_Server(ipAddr, port);
	if (clientSocket)
    	cout << "TCP server setup OK!" << endl;
	else
		cout << "Failed to setup TCP server!" << endl;

	// Main loop
	while (true) {
		// RECEIVE AND PARSE SERIAL DATA
		if (serialRXenable) {
			serialAvail = serial_data_available(pi, serialHdl);
			printf("Serial available: %d\n", serialAvail);
			if (serialAvail >= serialMaxBytes)
				serialRXbyteCount = serial_read(pi, serialHdl, serialBuf, sizeof(serialBuf));
			printf("RX: %s\n", serialBuf);
		}
		// RECEIVE AND PARSE SOCKET DATA
		memset(socketBuf, 0, sizeof(socketBuf));													// clear receive buffer
        TCPRXbyteCount = recv(clientSocket, socketBuf, sizeof(socketBuf), MSG_DONTWAIT);
		if (TCPRXbyteCount == 0) {
			cerr << "There was a connection issue. Waiting for reconnection..." << endl;
			// DISABLE MOTORS IF CONNECTION IS LOST
			toggle_autopilot(false);
			disable_motors(pi);
			// ENGAGE BRAKES
			set_servo_pulsewidth(pi, brake_servo, brakeFull);	// brake fully to prevent rolling down a hill
			run_brake_lights(100, pi, lightingHdl);				// turn on brake lights
			goto loop;
		}
		printf("recv()'d %d bytes of data in buf\n", TCPRXbyteCount);
		printf("Received: %s\n", socketBuf);

		gpio_write(pi, 19, 1);		// enable steering motor when program enters normal loop

		// two possible sources: serialBuf, socketBuf
		// fill preParseBuf with selected data

		// always parse button set 2 from socket to toggle manual/lane assist control
		CVenable = get_cv_flag(serialRXenable, socketBuf);
		run_cv_status_led(CVenable, pi, lightingHdl);	// set status LED according to CVenable

		printf("CVenable: %d\n", CVenable);

		if (CVenable) {
			// SERIAL INPUT MODE
			if (serialRXbyteCount == serialMaxBytes && serialRXenable) {
				dataReady = 1; // mark data ready
				// if true, copy serialBuf into preParseBuf
				for (int i = 0; i < serialMaxBytes; i++) {
					preParseBuf[i] = serialBuf[i];
				}
			} else
				// if received not enough bytes or serial is disabled, mark data not ready
				dataReady = 0;
		} else {
			// SOCKET INPUT MODE
			if (TCPRXbyteCount >= serialMaxBytes + 1 && TCPRXbyteCount <= 54) {
				// check for valid received byte count
				dataReady = 1; // mark data ready
				// if true, copy socketBuf into preParseBuf
				for (int i = 0; i < serialMaxBytes + 1; i++) 
		 			preParseBuf[i] = socketBuf[i];
			} else
				 // if received not enough bytes, mark data not ready
				 dataReady = 0;
		}

		printf("dataReady: %d\n", dataReady);
		printf("preParseBuf: %s\n", preParseBuf);

		parse_control_data(dataReady, preParseBuf, gear, steerValue, accelValue, brakeValue, buttons1, buttons2);

		printf("Gear: %d\n", gear);
		printf("Steering: %d\n", steerValue);
		printf("Acceleration: %d\n", accelValue);
		printf("Braking: %d\n", brakeValue);
		printf("Button set 1: %d\n", buttons1);
		printf("Button set 2: %d\n", buttons2);
		// printf("Distance From Lidar: %d\n", lidarDist);

		// (DEPRECIATED CODE COMMENTED OUT)
		// // ACTIVE SAFETY (DEPRECIATED)
		// if (ASenable) {
		// 	printf("AS_STATE: %d\n", AS_state);
		// 	switch (AS_state) {
		// 	case 0: // normal state
		// 		haltFlag = 0;
		// 		if (run_active_safety(pi, ultrasonicHdl))
		// 			AS_state = 1;	// move to next state
		// 		else
		// 			AS_state = 0;	// stay
		// 	break;
		// 	case 1: // obstacle detected
		// 		run_active_safety(pi, ultrasonicHdl);		// only used to print ultrasonic data
		// 		haltFlag = 1;
		// 		if (accelValue == 0)
		// 			AS_state = 2;	// if accel pedal is released, move to next state
		// 		else
		// 			AS_state = 1;	// stay
		// 	break;
		// 	case 2: // pedal released, continue halting
		// 		run_active_safety(pi, ultrasonicHdl);		// only used to print ultrasonic data
		// 		haltFlag = 1;
		// 		if (accelValue > 0)
		// 			AS_state = 3;	// if accel pedal is presesd, move to next state
		// 		else
		// 			AS_state = 2;	// stay
		// 	break;
		// 	case 3:	// un-halt and start timer
		// 		run_active_safety(pi, ultrasonicHdl);		// only used to print ultrasonic data
		// 		haltFlag = 0;
		// 		AS_timer++;
		// 		printf("AS_TIMER: %d\n", AS_timer);
		// 		if (AS_timer > 12) {
		// 			AS_timer = 0;	// clear timer
		// 			if (run_active_safety(pi, ultrasonicHdl))
		// 				AS_state = 1;	// if obstacle is still present, move to state 1
		// 			else
		// 				AS_state = 0;	// if obstacle is not present, move to state 0
		// 		} else
		// 			AS_state = 3;	// stay
		// 	break;
		// 	}
		// }

		// if (haltFlag) {
		// 	controlledAccelValue = 0;
		// 	controlledBrakeValue = 100;
		// 	// brake
		// } else {
		// 	controlledAccelValue = accelValue;
		// 	controlledBrakeValue = brakeValue;
		// }

		// printf("haltFlag: %d\n", haltFlag);
		
		// DO MAIN FUNCTIONS HERE
		controlledAccelValue = accelValue;
		controlledBrakeValue = brakeValue;

		// AUTOPILOT
		lidarDist = run_lidar(buttons2, pi, lidarHdl, lidarAddr);
		if (cruise_active())
		{
			controlledAccelValue = calculate_cruise(lidarDist);
			controlledBrakeValue = calculate_cruise_breaks(lidarDist);
		}

		if (controlledBrakeValue > 0) { controlledAccelValue = 0; }

		// CONTROL ACCELERATION, STEERING, SERVOS, AND SAMPLE ADC
		run_acceleration(pi, controlledAccelValue, gear);

		set_servo_pulsewidth(pi, camera_servo, map(run_camera_pan(buttons1), 3, 25, 600, 2400));
		set_servo_pulsewidth(pi, brake_servo, map(controlledBrakeValue, 0, 100, brakeRelaxed, brakeFull)); // relaxed | braking 2190 old bat
		spi_xfer(pi, adcHdl, spi_tx, spi_rx, 3);
		adc_data = (spi_rx[1] << 8) | spi_rx[2] & 0x3FF;
		printf("Actual steering position: %d\n", adc_data);

		gps_destination_counter += run_autopilot(buttons2, run_GPS(pi, GPSHdl, GPSAddr), destination_data[gps_destination_counter]);
		if (autopilot_active())
		{
			// Counter will increase when GPS's longitude and latitude line up with destination_data's
			cout << "gps_destination_counter: " << gps_destination_counter << "\n";

			// Steer the car based on the course angle and distance
			steerValue = calculate_autopilot_steering(run_GPS(pi, GPSHdl, GPSAddr), destination_data[gps_destination_counter]);
			
		}


		run_steering(deadband, pi, steerValue, adc_data);
		run_brake_lights(controlledBrakeValue, pi, lightingHdl);
		run_reverse_lights(gear, pi, lightingHdl);
		run_turn_signals(buttons2, pi, lightingHdl);
		run_headlights(buttons2, pi, lightingHdl);

		
		printf("\n");
		usleep(50*1000);
    }
    return 0;
}



void signal_callback_handler(int signum) {
   cout << "CTRL+C caught! Terminating..." << signum << endl;
   toggle_autopilot(false);
   disable_motors(pi);
   set_servo_pulsewidth(pi, brake_servo, brakeRelaxed);	// relax brake servo
   run_brake_lights(0, pi, lightingHdl);				// turn off brake lights
   serial_close(pi, serialHdl);
   i2c_close(pi, lightingHdl);
   i2c_close(pi, ultrasonicHdl);
   i2c_close(pi, lidarHdl);
   i2c_close(pi, GPSHdl);
   spi_close(pi, adcHdl);

   pigpio_stop(pi);

   close(clientSocket);

   //system("sudo killall pigpiod");
   
   // Terminate program
   exit(signum);
}
