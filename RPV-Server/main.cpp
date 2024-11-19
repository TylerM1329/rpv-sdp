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
// Signal handler for clean shutdown
void signal_callback_handler(int signum);

// Pin and subsystem addresses (AVOID using addresses LESS THAN 10!)
const int camera_servo = 18;                 // Camera servo GPIO pin
const int brake_servo = 13;                  // Brake servo GPIO pin
const int ultrasonicAddr = 3;                // I2C address for ultrasonic subsystem
const int lightingAddr = 6;                  // I2C address for lighting subsystem
const int lidarAddr = 0x20;                  // I2C address for Lidar subsystem
const int GPSAddr = 0x15;                    // I2C address for GPS subsystem

// Global handles
int pi = 0;                                  // pigpio handle
int ultrasonicHdl = 0;                       // Ultrasonic I2C handle
int lightingHdl = 0;                         // Lighting I2C handle
int lidarHdl = 0;                            // Lidar I2C handle
int GPSHdl = 0;                              // GPS I2C handle
int clientSocket = 0;                        // TCP client socket
int adcHdl = 0;				     			 // SPI ADC handle
int serialHdl = 0;							 // Serial handle


// Servo and braking constants
const int brakeRelaxed = 2050;               // Relaxed brake servo position
const int brakeFull = 2250;                  // Fully engaged brake servo position
const int serialMaxBytes = 17;	      	     // Max bytes of string for processing in bus

// GPS waypoint array
double destination_data[6][3] = {
	// 2D array to hold GPS datapoints [longitude, latitude, course angle]
	{34.11457825, -117.70418549,0},      // point 1
	{34.11465073,-117.70417786,0},       // point 2
	{34.11465454,-117.70425415,0}, 	     // point 3
	{-1,-1,-1}                           // End of GPS waypoints
};

int gps_destination_counter = 0;             // Tracks the current waypoint

int main() {
	// Network Configuration
	char ipAddr[15];                              // IP address for TCP server
	int port = 0;                                 // Port for TCP server
	char socketBuf[512];                          // Receive buffer for TCP socket
	int TCPRXbyteCount = 0;                       // Byte count of received TCP message
	
	// Serial Communication
	char serialBuf[serialMaxBytes + 1];           // Receive buffer for serial port
	int serialRXbyteCount = 0;                    // Byte count of received serial message
	int serialAvail = 0;                          // Number of available bytes in serial buffer
	int serialRXenable = 0;                      // Enable/disable serial communication
	
	// Control States
	bool CVenable = 0;                            // Computer vision lane-keeping option
	int ASenable = 0;                             // Active safety option
	bool dataReady = 0;                           // Flag indicating if data is ready for parsing
	
	// Control Data
	int steerValue = 0;                           // Parsed steering value
	int accelValue = 0;                           // Parsed acceleration value
	int brakeValue = 0;                           // Parsed braking value
	int controlledAccelValue = 0;                 // Adjusted acceleration after safety or autopilot checks
	int controlledBrakeValue = 0;                 // Adjusted braking after safety or autopilot checks
	int buttons1 = 0;                             // Button set 1 (e.g., X and B) for camera panning
	int buttons2 = 0;                             // Button set 2 (e.g., left/right buttons) for turn signals, CV toggle
	int gear = 0;                                 // Gear flag: 0 = forward, 1 = reverse
	
	// Sensors and Subsystems
	int lidarDist = 0;                            // Lidar distance value
	int GPSLoca = 0;                              // GPS data placeholder
	char usSensors[4];                            // Array for ultrasonic sensor distances (1 byte per sensor)
	char preParseBuf[serialMaxBytes + 1];         // Buffer for data entering parsing logic
	
	// SPI Configuration
	char spi_tx[3] = {1, 128, 0};                 // Command to read channel 0 from MCP3008
	char spi_rx[3];                               // Array to hold received SPI data
	int adc_data = 0;                             // ADC data for steering feedback
	
	// Steering Logic
	int deadband = 11;                            // Deadband value for steering adjustments

	// Initialize hardware subsystems
	if (!init_IO(pi, ultrasonicHdl, ultrasonicAddr, lightingHdl, lightingAddr, adcHdl, serialHdl, lidarHdl, lidarAddr, GPSHdl, GPSAddr))
		printf("[!!] One or more I/O devices failed to initialize!\n");
	
	// Setup signal handler for graceful shutdown
	signal(SIGINT, signal_callback_handler);

	// Disable motors and set initial states
	disable_motors(pi);
	set_servo_pulsewidth(pi, brake_servo, brakeRelaxed); // Relax brake servo
	run_brake_lights(0, pi, lightingHdl);               // Turn off brake lights

	// User and network configuration
	get_network_options(ipAddr, port);		// Prompt user for network settings
	printf("Accepted values: IP = %s Port = %d", ipAddr, port);
	get_user_options(serialRXenable, ASenable);	// Prompt user for mode selection
	cout << "Opening Socket! Listening at " << ipAddr << ":" << port << endl;

	// TCP server loop
	loop:
	// Call TCP server setup function and store socket number
	clientSocket = setup_TCP_Server(ipAddr, port);
	if (clientSocket)
    	cout << "TCP server setup OK!" << endl;
	else
		cout << "Failed to setup TCP server!" << endl;

	// Main control loop
	while (true) {
		// --- RECEIVE AND PARSE SERIAL DATA ---
		if (serialRXenable) {
			// Check if there are available bytes in the serial buffer
			serialAvail = serial_data_available(pi, serialHdl);
			printf("Serial available: %d\n", serialAvail);
			
			// If sufficient data is available, read it into the serial buffer
			if (serialAvail >= serialMaxBytes)
				serialRXbyteCount = serial_read(pi, serialHdl, serialBuf, sizeof(serialBuf));
			printf("RX: %s\n", serialBuf);
		}
		
		// --- RECEIVE AND PARSE SOCKET DATA ---
		// Clear the TCP socket receive buffer
		memset(socketBuf, 0, sizeof(socketBuf)); // clear receive buffer
		
		// Attempt to receive data from the TCP socket in non-blocking mode
        	TCPRXbyteCount = recv(clientSocket, socketBuf, sizeof(socketBuf), MSG_DONTWAIT);
		
		// If no data is received, handle connection loss
		if (TCPRXbyteCount == 0) {
			cerr << "There was a connection issue. Waiting for reconnection..." << endl;
			
			// Disable motors and engage brakes for safety
			toggle_autopilot(false);				// Stop autopilot
			disable_motors(pi);
			set_servo_pulsewidth(pi, brake_servo, brakeFull);	// Apply full brakes
			run_brake_lights(100, pi, lightingHdl);			// Turn on brake lights
			
			// Reattempt connection
			goto loop;
		}
		printf("recv()'d %d bytes of data in buf\n", TCPRXbyteCount);
		printf("Received: %s\n", socketBuf);

		// Ensure steering motor is enabled
		gpio_write(pi, 19, 1);

		// --- SELECT DATA SOURCE (SERIAL OR SOCKET) ---
		// Always check button set 2 from the socket for manual/auto lane assist control
		CVenable = get_cv_flag(serialRXenable, socketBuf);
		run_cv_status_led(CVenable, pi, lightingHdl);	// Update status LED for lane assist mode
		printf("CVenable: %d\n", CVenable);

		if (CVenable) {
			// --- SERIAL INPUT MODE ---
			if (serialRXbyteCount == serialMaxBytes && serialRXenable) {
				dataReady = 1;	// Mark data as ready
				// Copy serial buffer data into parsing buffer
				for (int i = 0; i < serialMaxBytes; i++) {
					preParseBuf[i] = serialBuf[i];
				}
			} else {
				dataReady = 0; 	// Mark data as not ready if insufficient bytes or serial is disabled
			}
		} else {
			// --- SOCKET INPUT MODE ---
			if (TCPRXbyteCount >= serialMaxBytes + 1 && TCPRXbyteCount <= 54) {
				// check for valid received byte count
				dataReady = 1;	// Mark data as ready
				// Copy socket buffer data into parsing buffer
				for (int i = 0; i < serialMaxBytes + 1; i++) {
		 			preParseBuf[i] = socketBuf[i];
				}
			} else {
				dataReady = 0; 	// Mark data as not ready if insufficient bytes or serial is disabled
			}
		}

		printf("dataReady: %d\n", dataReady);
		printf("preParseBuf: %s\n", preParseBuf);
		
		// Parse control data into actionable variables
		parse_control_data(dataReady, preParseBuf, gear, steerValue, accelValue, brakeValue, buttons1, buttons2);
		printf("Gear: %d\n", gear);
		printf("Steering: %d\n", steerValue);
		printf("Acceleration: %d\n", accelValue);
		printf("Braking: %d\n", brakeValue);
		printf("Button set 1: %d\n", buttons1);
		printf("Button set 2: %d\n", buttons2);
		
		// --- PERFORM MAIN CONTROL FUNCTIONS ---
	
		// --- CRUISE CONTROL ---
		lidarDist = run_lidar(buttons2, pi, lidarHdl, lidarAddr);
		if (cruise_active()) {
			// Adjust acceleration and braking based on Lidar distance
			controlledAccelValue = calculate_cruise(lidarDist);
			controlledBrakeValue = calculate_cruise_breaks(lidarDist);
		} else {
			// Use manual control inputs for acceleration and braking
			controlledAccelValue = accelValue;
			controlledBrakeValue = brakeValue;
		}

		// Ensure braking overrides acceleration
		if (controlledBrakeValue > 0) { controlledAccelValue = 0; }

		// --- CONTROL VEHICLE SYSTEMS ---
		run_acceleration(pi, controlledAccelValue, gear);
		set_servo_pulsewidth(pi, camera_servo, map(run_camera_pan(buttons1), 3, 25, 600, 2400)); // Control camera panning
		set_servo_pulsewidth(pi, brake_servo, map(controlledBrakeValue, 0, 100, brakeRelaxed, brakeFull)); // Control braking
		
		// Sample ADC data for steering feedback
		spi_xfer(pi, adcHdl, spi_tx, spi_rx, 3);
		adc_data = (spi_rx[1] << 8) | spi_rx[2] & 0x3FF;
		printf("Actual steering position: %d\n", adc_data);

		// --- AUTOPILOT CONTROL ---
		gps_destination_counter += run_autopilot(buttons2, run_GPS(pi, GPSHdl, GPSAddr), destination_data[gps_destination_counter]);
		
		if (autopilot_active())
		{
			// Adjust steering based on GPS waypoints
			cout << "gps_destination_counter: " << gps_destination_counter << "\n"; // Current waypoint
			steerValue = calculate_autopilot_steering(run_GPS(pi, GPSHdl, GPSAddr), destination_data[gps_destination_counter]);
		}

		// --- UPDATE HARDWARE ---
		run_steering(deadband, pi, steerValue, adc_data);		// Update steering motor
		run_brake_lights(controlledBrakeValue, pi, lightingHdl);	// Update brake lights
		run_reverse_lights(gear, pi, lightingHdl);			// Update reverse lights
		run_turn_signals(buttons2, pi, lightingHdl);			// Update turn signals
		run_headlights(buttons2, pi, lightingHdl);			// Update headlights

		// Loop delay to prevent CPU overload
		printf("\n");
		usleep(50*1000);	// Delay for 50ms
    }
    return 0;
}

// Handles CTRL+C (SIGINT) shutdowns
void signal_callback_handler(int signum) {
	cout << "CTRL+C caught! Terminating..." << signum << endl;
	
	// Disable autonomous features
	toggle_autopilot(false);
	
	// Safely stop vehicle movement
	disable_motors(pi);	// Disable all motor outputs
	set_servo_pulsewidth(pi, brake_servo, brakeRelaxed);	// Relax the brake servo to avoid locking the wheels
	
	// Turn off all lights for safety
	run_brake_lights(0, pi, lightingHdl);	// Ensure brake lights are turned off

	// Close serial communication
	serial_close(pi, serialHdl);	// Close the serial port

	// Close all I²C handles to free resources
	i2c_close(pi, lightingHdl);	// Close lighting I²C handle
	i2c_close(pi, ultrasonicHdl);	// Close ultrasonic I²C handle
	i2c_close(pi, lidarHdl);	// Close Lidar I²C handle
	i2c_close(pi, GPSHdl);		// Close GPS I²C handle

	// Close SPI communication
	spi_close(pi, adcHdl);		// Close SPI ADC handle

	// Disconnect from pigpio daemon
	pigpio_stop(pi);		// Shut down pigpio library and release GPIO control

	// Close the TCP client socket
	close(clientSocket);		// Safely close the active TCP connection
   
	// Exit the program gracefully with the signal code
	exit(signum);
}