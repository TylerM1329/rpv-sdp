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
#include <math.h>
#include <cmath>

using namespace std;

// --- Mathematical Constants ---
#define PI 3.14159265

// --- Lidar and Buffer Configuration ---
int cm_buffer = 200;

// --- Drive Motor Pin Assignments ---
const int drive_RPWM = 17;	// GPIO pin for controlling the drive motor's forward PWM signal
const int drive_LPWM = 27;	// GPIO pin for controlling the drive motor's reverse PWM signal
const int drive_REN = 21;	// GPIO pin for enabling the drive motor in reverse
const int drive_LEN = 20;	// GPIO pin for enabling the drive motor in forward

// --- Steering Motor Pin Assignments ---
const int steering_RPWM = 24;	// GPIO pin for controlling the steering motor's right-turn PWM signal
const int steering_LPWM = 23;	// GPIO pin for controlling the steering motor's left-turn PWM signal
const int steering_EN = 19;	// GPIO pin for enabling the steering motor

// --- Cruise Control Variables ---
int cruise_activated = 0;
int cruise_toggle = 0;

// --- Autopilot Variables ---
int autopilot_activated = 0;
int autopilot_toggle = 0;

// --- Speed Configuration ---
int MAX_SPEED = 100;

// Maps a value from one range to another range using a linear transformation
// Parameters:
// - long x: The input value to be mapped
// - long in_min: The lower bound of the input range
// - long in_max: The upper bound of the input range
// - long out_min: The lower bound of the output range
// - long out_max: The upper bound of the output range
// Returns:
// - long: The mapped value in the output range
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Controls the camera pan position based on directional input
// - int direction: Input direction for camera panning
//     1 = Pan left
//     2 = Pan right
//     3 = Center the camera
int run_camera_pan(int direction) {
	// Static variable to maintain the current camera position across calls
	static int currPos = 14; // Default starting position is centered (14)

	// --- Handle Directional Input ---
	if (direction == 1) { // Pan left (increment position)
		if (currPos < 25) // Ensure position doesn't exceed maximum
			currPos++; // Increment the current position
	} else if (direction == 2) { // Pan right (decrement position)
		if (currPos > 3) // Ensure position doesn't go below minimum
			currPos--; // Decrement the current position
	} else if (direction == 3) { // Center the camera
		currPos = 14; // Reset position to center
	}
	return currPos; // Return the updated camera position
}

// Controls the vehicle's acceleration and drive motor state based on the current acceleration value and gear
void run_acceleration(int pi, int intAccel, int gear) {
	// --- Calculate Acceleration ---
	if (intAccel) { // Only map acceleration if a non-zero value is provided
		// Autopilot, control acceleration automatically based on lidar
		if (autopilot_active())	{
			// In autopilot mode, map the acceleration value proportionally to a PWM range (0-255)
			intAccel = map(intAccel, 0, 100, 0, 255);
		} else {
			// In manual mode, set the acceleration to a fixed maximum speed
			intAccel = map(MAX_SPEED, 0, 100, 0, 255);
		}
	}

	// --- Handle Drive Motor States ---
	if (intAccel) { // Acceleration is active
		if (gear == 1) { // Reverse gear
			printf("Drive motor engaged\n");
			set_mode(pi, drive_REN, PI_OUTPUT);		// Configure reverse enable pin as output
			set_mode(pi, drive_LEN, PI_OUTPUT);		// Configure forward enable pin as output
			gpio_write(pi, drive_REN, 1); 			// Enable reverse (both pins set high)
			gpio_write(pi, drive_LEN, 1); 			// Enable forward (both pins set high)
			set_PWM_dutycycle(pi, drive_RPWM, 0); 		// Set forward PWM to 0
			set_PWM_dutycycle(pi, drive_LPWM, intAccel); 	// Set reverse PWM to match acceleration
		} else if (gear == 0) {	// Forward gear
			printf("Drive motor engaged\n");
			set_mode(pi, drive_REN, PI_OUTPUT);		// Configure reverse enable pin as output
			set_mode(pi, drive_LEN, PI_OUTPUT);		// Configure forward enable pin as output
			gpio_write(pi, drive_REN, 1); 			// Enable reverse (both pins set high)
			gpio_write(pi, drive_LEN, 1); 			// Enable forward (both pins set high)
			set_PWM_dutycycle(pi, drive_RPWM, intAccel); 	// Set forward PWM to match acceleration
			set_PWM_dutycycle(pi, drive_LPWM, 0); 		// Set reverse PWM to 0
		}
	} else { // Acceleration is inactive (coasting mode)
			printf("Drive motor coasting\n");			
			set_mode(pi, drive_REN, PI_OUTPUT);		// Configure reverse enable pin as output
			set_mode(pi, drive_LEN, PI_OUTPUT);		// Configure forward enable pin as output
			gpio_write(pi, drive_REN, 0); 			// Disable reverse
			gpio_write(pi, drive_LEN, 0); 			// Disable forward
			set_PWM_dutycycle(pi, drive_RPWM, 0); 		// Set forward PWM to 0
			set_PWM_dutycycle(pi, drive_LPWM, 0); 		// Set reverse PWM to 0
		}
}

// Sets up a TCP server that listens for client connections on the specified IP and port
int setup_TCP_Server(char ip[], int port) {
	int clientSocket; // Variable to hold the client socket descriptor

loop:
	// --- Create a Listening Socket ---
	int listening = socket(AF_INET, SOCK_STREAM, 0); // Create a socket for IPv4 and TCP
	if (listening == -1) { // Check if socket creation failed
		cerr << "Can't create socket";
		return -1; // Return error code
	}

	// --- Configure the Socket Address ---
	sockaddr_in hint;           		// Structure to hold server address details
	hint.sin_family = AF_INET;  		// Specify IPv4
	hint.sin_port = htons(port);		// Convert port number to network byte order
	inet_pton(AF_INET, ip, &hint.sin_addr); // Convert IP address to binary format

	// --- Bind the Socket to the IP and Port ---
	if (bind(listening, (sockaddr*)&hint, sizeof(hint)) == -1) { // Attempt to bind the socket
		cerr << "Can't bind to IP/Port";
		goto loop; // Retry the setup process if binding fails
	}

	// --- Mark the Socket for Listening ---
	if (listen(listening, SOMAXCONN) == -1)	{ // Attempt to listen for incoming connections
		cerr << "Can't listen";
		return -3; // Return error code
	}

	// --- Accept a Client Connection ---
	sockaddr_in client;     // Structure to hold client address details
	socklen_t clientSize = sizeof(client);
	char host[NI_MAXHOST];	// Buffer to store client's hostname
	char svc[NI_MAXSERV];	// Buffer to store client's service name

	clientSocket = accept(listening, (sockaddr*)&client, &clientSize); // Accept an incoming connection
	close(listening); // Close the listening socket after accepting a client

	// --- Resolve Client Information ---
	memset(host, 0, NI_MAXHOST);	// Clear the host buffer
	memset(svc, 0, NI_MAXSERV);	// Clear the service buffer
	int result = getnameinfo((sockaddr*)&client, clientSize, host, NI_MAXHOST, svc, NI_MAXSERV, 0);

	if (result) {
		// If hostname resolution is successful, print the hostname and service
		cout << host << " connected on " << svc << " connection 1"<<endl;
	}
	else {
		// If hostname resolution fails, fall back to raw IP and port
		inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
		cout << host << " connected on " << ntohs(client.sin_port) << " connection 2"<<endl;
	}
	cout << "EXITING TCP STARTUP" << endl;
	return clientSocket; // Return the client socket descriptor
}

// Controls the vehicle's brake lights based on the brake value
void run_brake_lights(int brakeValue, int piHandle, int lightingHandle) {
	// Static variables to maintain debounce state across function calls
	static bool brakeFlag1 = 1;
	static bool brakeFlag2 = 1;

	// --- Handle Brake Light Activation ---
	if (brakeValue > 50) { // Threshold for activating brake lights (adjust as needed)
		brakeFlag2 = 1; // Reset the deactivation debounce flag
		if (brakeFlag1) { // Only send the activation command if the debounce flag is set
			i2c_write_byte(piHandle, lightingHandle, 6); // Command to activate brake lights
			brakeFlag1 = 0; // Clear the activation debounce flag to prevent repeated commands
		}
	} 
	// --- Handle Brake Light Deactivation ---
	else { // Brake value is below the threshold
		brakeFlag1 = 1; // Reset the activation debounce flag
		if (brakeFlag2) { // Only send the deactivation command if the debounce flag is set
			i2c_write_byte(piHandle, lightingHandle, 7); // Command to deactivate brake lights
			brakeFlag2 = 0; // Clear the deactivation debounce flag to prevent repeated commands
		}
	}
}

// Controls the vehicle's reverse lights based on the current gear state
void run_reverse_lights(int gear, int piHandle, int lightingHandle) {
	// Static variables to maintain debounce state across function calls
	static bool gearFlag1 = 1;
	static bool gearFlag2 = 1;

	// --- Handle Reverse Light Activation ---
	if (gear) { // Gear is in reverse (gear == 1)
			gearFlag2 = 1; // Reset the debounce flag for light deactivation
			if (gearFlag1) { // Only send the activation command if the debounce flag is set
				// Uncomment and replace `6` with the correct I²C command for activating reverse lights
				//i2c_write_byte(piHandle, lightingHandle, 6);
				gearFlag1 = 0; // Clear the activation debounce flag to prevent repeated commands
			}
	} else {
		gearFlag1 = 1; // Reset the debounce flag for light activation
		if (gearFlag2) { // Only send the deactivation command if the debounce flag is set
			// Uncomment and replace `7` with the correct I²C command for deactivating reverse lights
			//i2c_write_byte(piHandle, lightingHandle, 7); // change 7 to something else
			gearFlag2 = 0; // Clear the deactivation debounce flag to prevent repeated commands
		}
	}
}

// Controls the vehicle's turn signals based on button inputs
void run_turn_signals(int buttons2, int piHandle, int lightingHandle) {
	// Debugging output: Log the current button state for turn signals
	cout << "Started running turn signal " << buttons2 << endl;

	// Static variables to maintain debounce state across function calls
	static bool turnFlag1 = 1;
	static bool turnFlag2 = 1;

	// --- Handle Left Turn Signal ---
	if (buttons2 == 1) { // Button 1 indicates left turn signal activation
		turnFlag2 = 1; // Reset the debounce flag for the right turn signal
		if (turnFlag1) { // Only send the command if the debounce flag is set
			i2c_write_byte(piHandle, lightingHandle, 2); // Command to activate the left turn signal
			turnFlag1 = 0; // Clear the debounce flag to prevent repeated commands
		}
	}
	// --- Handle Right Turn Signal ---
	else if (buttons2 == 2) { // Button 2 indicates right turn signal activation
		turnFlag1 = 1; // Reset the debounce flag for the left turn signal
		if (turnFlag2) { // Only send the command if the debounce flag is set
			i2c_write_byte(piHandle, lightingHandle, 4); // Command to activate the right turn signal
			turnFlag2 = 0; // Clear the debounce flag to prevent repeated commands
		}
	} 
	// --- Default State (No Turn Signal) ---
	else {
		// Reset both debounce flags to their initial state when no button is pressed
		turnFlag1 = 1;
		turnFlag2 = 1;
	}

	// Debugging output: Indicate that turn signal processing is complete
	cout << "Done running turn signal" << endl;
}

// Controls the status LED for computer vision (CV) mode based on its activation state
void run_cv_status_led(int status, int piHandle, int lightingHandle) {
	// Static variables to maintain state across function calls
	static bool flag1 = 1; // Debounce flag to prevent redundant ON commands
	static bool flag2 = 1; // Debounce flag to prevent redundant OFF commands

	// --- Handle CV Status LED ---
	if (status == 1) { // CV mode is enabled
		flag2 = 1; // Reset the OFF debounce flag
		if (flag1) { // Only send the ON command if the debounce flag is set
			i2c_write_byte(piHandle, lightingHandle, 8); // Send command to turn the status LED ON
			flag1 = 0; // Clear the ON debounce flag to prevent repeated commands
		}
	} else if (status == 0) { // CV mode is disabled
		flag1 = 1; // Reset the ON debounce flag
		if (flag2) { // Only send the OFF command if the debounce flag is set
			i2c_write_byte(piHandle, lightingHandle, 9); // Send command to turn the status LED OFF
			flag2 = 0; // Clear the OFF debounce flag to prevent repeated commands	
		}
	} else { // Default state for safety or invalid status values
		// Reset both debounce flags to their initial state
		flag1 = 1;
		flag2 = 1;
	}
}

// Controls the headlights based on the toggle input
void run_headlights(int toggle, int piHandle, int lightingHandle) {
	// Static variables to maintain state across function calls
	static bool headFlag1 = 1;	// Debounce flag to prevent rapid toggling
	static bool headFlag2 = 1;	// Secondary debounce flag for reset
	static bool headlightEn = 0;	// State of the headlights (0 = off, 1 = on)

	// --- Toggle Headlights ---
	if (toggle == 4) { // Check if the toggle signal is received
		headFlag2 = 1; // Reset the secondary debounce flag
		if (headFlag1) { // Only execute if the primary debounce flag is set
			headlightEn = !headlightEn; // Toggle the headlight state

			// Send the appropriate I²C command based on the new headlight state
			if (headlightEn) {
				printf("ENABLE HEADLIGHT CMD SENT");
				i2c_write_byte(piHandle, lightingHandle, 0); // Send command to turn headlights ON
			} else {
				printf("DISABLE HEADLIGHT CMD SENT");
				i2c_write_byte(piHandle, lightingHandle, 1); // Send command to turn headlights OFF
			}
			headFlag1 = 0; // Clear the primary debounce flag to prevent immediate re-triggering
		}
	} else { // Reset debounce flags when the toggle signal is not active
		headFlag1 = 1; // Reset primary debounce flag
		if (headFlag2) {
			headFlag2 = 0; // Reset secondary debounce flag
		}
	}
}

// Controls the steering motor to adjust the steering angle based on the target position
void run_steering(int deadband, int piHandle, int targetPos, int currentPos) {
	// --- Initialize Steering Parameters ---
	int steeringDutyCycle = 255; // Maximum PWM duty cycle (0-255)
	// Note: High duty cycles may cause oscillations; 180 is suitable for most cases.

	// Map the target position from a 0-100 range to a PWM-compatible range
	targetPos = map(targetPos, 0, 100, 90, 417);

	// --- Steering Control Logic ---
	// Check if the current position is within the acceptable deadband around the target
	if ((currentPos <= targetPos + deadband) && (currentPos >= targetPos - deadband)) {
		// Centered position: No adjustments required
		printf("Steering: CENTERED\n");
		set_PWM_dutycycle(piHandle, steering_RPWM, 0); 
		set_PWM_dutycycle(piHandle, steering_LPWM, 0);
    	}
	// Check if the current position is to the right of the target (needs left turn)
	else if (currentPos > targetPos + deadband) {
		printf("Steering: TURNING RIGHT\n");
		set_PWM_dutycycle(piHandle, steering_RPWM, steeringDutyCycle);	// Engage right-turn motor
		set_PWM_dutycycle(piHandle, steering_LPWM, 0); 			// Disable left-turn motor
	}
	// Check if the current position is to the left of the target (needs right turn)
	else if (currentPos < targetPos - deadband) {
		printf("Steering: TURNING LEFT\n");
		set_PWM_dutycycle(piHandle, steering_RPWM, 0);			// Disable right-turn motor
		set_PWM_dutycycle(piHandle, steering_LPWM, steeringDutyCycle);	// Engage left-turn motor
	}
}

// DEPRECIATED
int run_active_safety(int piHandle, int ultrasonicHandle) {
	static char usSensors[2];
	int threshold = 35;
	i2c_read_device(piHandle, ultrasonicHandle, usSensors, 2);
	/*for (int i = 0; i < 4; i++)
		printf("Sensor %d: %d\n", i, usSensors[i]);*/
	printf("Front: %dcm\n", usSensors[0]);
	printf("Back: %dcm\n", usSensors[1]);
	if (usSensors[0] < threshold || usSensors[1] < threshold)
		return 1;
	return 0;
	/*
	0 = normal
	1 = halt motors and brake.
	*/
}

// Reads distance data from a Lidar sensor and manages cruise control activation
int run_lidar(int toggle, int piHandle, int lidarHandle, int lidarAddr) {
	// --- Handle Cruise Control Toggle ---
	if (cruise_toggle == false && toggle == 5) {
		// Toggle cruise control activation on button press
		cruise_activated = !cruise_activated; // More elegant if-else statement for toggling cruise_control
		cruise_toggle = true; // Prevent re-triggering until button is released
	}
	else if(toggle == 0) {
		// Reset toggle flag when button is released
		cruise_toggle = false;
	}

	// --- Initialize Variables for Lidar Data ---
	uint16_t distance;	// Variable to hold the calculated Lidar distance
    	char data[2];  		// Buffer to store the 2 bytes of raw distance data

	// --- Read Data from Lidar Sensor ---
	int bytesRead = i2c_read_device(piHandle, lidarHandle, data, 2);
	if (bytesRead == 2) {
		// Parse the distance from the 2-byte data buffer
		distance = (data[1] << 8) | data[0]; // Combine the two bytes into a 16-bit distance value
		cout << "Lidar Distance: " << distance << " cm" << std::endl;

		// If cruise control is not active, return -1 (no distance is used)
		if (!cruise_activated) {return -1;}
	}
	else {
		// --- Handle Lidar Read Errors ---
		// Log an error message and attempt to reinitialize the Lidar I²C handle
		cout << "Lidar failed. Reinitializing I2C handle...\n";
		i2c_close(piHandle, lidarHandle); // Close the existing Lidar handle
		lidarHandle = i2c_open(piHandle, 1, lidarAddr, 0); // Reopen the I²C handle for the Lidar
		return -1; // Return -1 to indicate an error occurred
	}

    return distance;  // Return the calculated Lidar distance (in cm)
}

// Reads GPS data from an I2C device and returns the parsed results
double* run_GPS(int piHandle, int GPSHandle, int GPSAddr) {
	static double results[3];	// Static array to store latitude, longitude, and course angle
	char data[10];  		// Buffer to store 10 bytes of raw data from the GPS device
	float latitude;			// Variable to hold the parsed latitude
	float longitude;		// Variable to hold the parsed longitude
	int16_t course_angle;		// Variable to hold the parsed course angle in hundredths of a degree

	// --- Read GPS Data from I2C ---
	// Attempt to read 10 bytes from the GPS I2C device
	int bytesRead = i2c_read_device(piHandle, GPSHandle, data, 10);
	if (bytesRead == 10) {
		// --- Parse GPS Data ---
        	// Extract the latitude (4 bytes) from the raw data buffer
        	memcpy(&latitude, &data[0], 4);
		
		// Extract the longitude (4 bytes) from the raw data buffer
        	memcpy(&longitude, &data[4], 4);

		// Extract the course angle (2 bytes) from the raw data buffer
        	memcpy(&course_angle, &data[8], 2);

		// Convert the course angle to a double and scale it back to degrees
        	double course = static_cast<double>(course_angle) / 100.0; // Convert to double and scale back

        	// Store the parsed values into the results array
        	results[0] = static_cast<double>(latitude);	// Latitude
        	results[1] = static_cast<double>(longitude);	// Longitude
        	results[2] = course;				// Course angle

        	// Debugging output (commented out)
        	// printf("Latitude: %.8f, Longitude: %.8f, Course: %.2f\n", results[0], results[1], results[2]);
    	} else {
		// --- Handle Read Errors ---
        	// If the number of bytes read is incorrect, log an error
        	fprintf(stderr, "Error: Expected 10 bytes, but got %d\n", bytesRead);

		// Reset the results array to indicate failure
        	results[0] = 0.0;
        	results[1] = 0.0;
        	results[2] = 0.0;

		// Reinitialize the GPS I2C handle to attempt recovery
		cout << "GPS failed. Reinitializing I2C handle...\n";
		i2c_close(piHandle, GPSAddr);
		GPSHandle = i2c_open(piHandle, 1, GPSAddr, 0);
    }
	// Return the static results array containing the parsed GPS data
	return results;
}

void get_network_options(char *ipAddr, int &port) {
	int source = 0;
connectionInput:
	cout << "What address should I listen on? 1 for local, 2 for VPN, 3 for custom: ";
	cin >> source;
	switch (source)
	{
		case 1: {
			sprintf(ipAddr, "192.168.5.132");
			port = 54000;
			break;
		}
		case 2: {
			sprintf(ipAddr, "10.8.0.2");
			port = 54000;
			break;
		}
		case 3: {
			cout << "Enter IP: ";
			cin >> ipAddr;
			cout << "\nEnter Port: ";
			cin >> port;
			break;
		}
		default: {
			cout << "Invalid input!" << endl;
			goto connectionInput;
		}
	}
}

// Prompts the user to set control and safety options for the system
void get_user_options(int &serialOption, int &ASoption) {
	int userOption = 0;	// Temporary variable to store user input
	
	// --- Prompt for Serial Control Option ---
userInput:
	cout << "\nUser Options: 1 for VRX/XBOX control only, 2 for VRX/XBOX + lane keep assist: ";
	cin >> userOption;

	// Process user input for serial communication option
	switch (userOption)
	{
		case 1: {
			serialOption = 0; // Disable serial communication for control
			cout << "> Serial port DISABLED" << endl;
			break;
		}
		case 2: {
			serialOption = 1; // Enable serial communication for control
			cout << "> Serial port ENABLED" << endl;
			break;
		}
		default: {
			cout << "Invalid input!" << endl;
			goto userInput; // Retry prompt if input is invalid
		}
	}
	
	// --- Prompt for Active Safety Option ---
userInput2:
//-----------------------------------------------------------------------------------------------------------*
	// Depreciated section: Active Safety is no longer functional
	// Defaulting userOption to 2 (Active Safety OFF)
	userOption = 2;

	// cout << "Active Safety: 1 for ON, 0 for OFF: ";
	// cin >> userOption;
	switch (userOption) {
		case 2: { // Active Safety is disabled (default behavior)
			ASoption = 0;
			cout << "> Active Safety DEPRECIATED" << endl;
			break;
		}
		case 1: { // Enable Active Safety (if functionality is restored in the future)
			ASoption = 1;
			cout << "> Active Safety ENABLED" << endl;
			break;
		}
		case 0: { // Explicitly disable Active Safety
			ASoption = 0;
			cout << "> Active Safety DISABLED" << endl;
			break;
		}
		default: {
			cout << "Invalid input!" << endl;
			goto userInput2; // Retry prompt if input is invalid
		}
	}

	// --- Prompt for Maximum Speed ---
	cout << "\nSet max speed (1-100): ";
	cin >> userOption;
	
	// Ensure the speed is within valid bounds
	while(userOption <= 0) {
		cout << "\nSet max speed (1-100): ";
		cin >> userOption;
	}

	if (userOption > 100) {
		userOption = 100; // Cap speed at 100 if user input exceeds the limit
	}
	
	MAX_SPEED = userOption; // Set the maximum speed
	cout << "Speed set to " << userOption << "\n";
}


void parse_control_data(int dataReady, char buffer[], int &gear, int &steerValue, int &accelValue, int &brakeValue, int &buttons1, int &buttons2) {
	// strtok() breaks string (buf) into a series of tokens using "-" delimiter
	// convert string to int with atoi
	char *token;
	// If data is ready, then proceed to parse
	if (dataReady) {							
			// parse forward / reverse
			token = strtok(buffer, "-");		
			gear = (token[0] == 'R');	// check if byte 0 of token contains R. R = 1, F = 0
			// parse steering
			token = strtok(NULL, "-");
			steerValue = atoi(token);
			// parse acceleration
			token = strtok(NULL, "-");
			accelValue = atoi(token);
			// parse braking
			token = strtok(NULL, "-");
			brakeValue = atoi(token);
			// parse button set 1
			token = strtok(NULL, "-");
			buttons1 = atoi(token);
			// parse button set 2
			token = strtok(NULL, "-");
			buttons2 = atoi(token);

			// Disable autopilot + cruise if accelerating or braking 
			if(accelValue > 0 || brakeValue > 0) toggle_autopilot(false);
		
			// Disable autopilot if steering is detected
			if(autopilot_activated && steerValue != 50) toggle_autopilot(false);
		}
}

// Handles toggling of the computer vision (CV) enable flag with debounce logic
int get_cv_flag(int enable, char buffer[]) {	// HARDCODED INDEX IN BUFFER!
	// Static variables to persist between function calls
	static bool CVbounceFlag = 0;
	static bool CVenable = 0;
	static int CVbounceTimer = 0;

	// Check if the control button is pressed (buffer[16] == '3') and debounce flag is not set
	if (buffer[16] == '3' && CVbounceFlag == 0) {	
		CVbounceFlag = 1;		// Set the bounce flag to prevent re-execution
		if (enable) {
			CVenable = !CVenable;	// Toggle the CV enable flag if lane-keeping mode is allowed
		}
	} else {
		// If the bounce flag is already set, increment the bounce timer
		CVbounceTimer++;
		//printf("timer = %d\n", CVbounceTimer);
		
		// Reset the bounce timer and flag after a debounce delay
		if (CVbounceTimer > 4) {	// if the timer has reached 2, reset the timer and the flag to allow CV control
			CVbounceTimer = 0;
			CVbounceFlag = 0;									
		}
	}
	// Return the current state of the CV enable flag
	return CVenable;
}

// Initializes I/O subsystems
int init_IO(int &piHandle, int &ultrasonicHandle, int ultrasonicAddr, int &lightingHandle, int lightingAddr, int &adcHandle, int &serialHandle, int &lidarHandle, int lidarAddr, int& GPSHandle, int GPSAddr) {
	bool status = true;	// Status flag to track initialization success

	// Initialize pigpio daemon
	piHandle = pigpio_start(NULL, NULL);
	if (piHandle < 0) {
		status = false;
		printf("[!] Failed to connect to pigpio daemon! Attempted to connect to localhost:8888\n");
	} else {
        	printf("> pigpio on localhost:8888 \tINIT OK\n");
	}
	
	// Initialize I²C for the ultrasonic subsystem
	ultrasonicHandle = i2c_open(piHandle, 1, ultrasonicAddr, 0);		// init I2C and assign handle to "ultrasonicHdl" 
	if (ultrasonicHandle < 0) {
		status = false;
		printf("[!] Failed to init ultrasonic subsystem!\n");
	} else {
		printf("> ultrasonic subsystem \t\tINIT OK\n");
	}

	// Initialize I²C for the lighting subsystem
	lightingHandle = i2c_open(piHandle, 1, lightingAddr, 0);			// init I2C and assign handle to "lightingHdl"
	if (lightingHandle < 0) {
		status = false;
		printf("[!] Failed to init lighting subsystem!\n");
	} else {
		printf("> lighting subsystem \t\tINIT OK\n");
	}

	// Initialize SPI for the ADC (Analog-to-Digital Converter)
	adcHandle = spi_open(piHandle, 0, 1000000, 0);						// init SPI and assign handle to "adc"
	if (adcHandle < 0) {
		status = false;
		printf("[!] Failed to init ADC!\n");
	} else {
		printf("> ADC \t\t\t\tINIT OK\n");
	}

	// Initialize the serial port
	char port[] = "/dev/ttyS0";
	serialHandle = serial_open(piHandle, port, 115200, 0);			// init serial and assign handle to "serialHdl"
	if (serialHandle < 0) {
		status = false;
		printf("[!] Failed to init serial! Try turning car off and on again!\n");
	} else {
		printf("> serial open %s \tINIT OK\n", port);
	}

	// Initialize I²C for the lidar subsystem
	lidarHandle = i2c_open(piHandle, 1, lidarAddr, 0);
	if (lidarHandle < 0) {
		status = false;
		printf("[!] Failed to init lidar!\n");
	} else {
		printf("> lidar subsystem \t\tINIT OK\n");
	}

	// Initialize I²C for the GPS subsystem
	GPSHandle = i2c_open(piHandle, 1, GPSAddr, 0);
	if (GPSHandle < 0) {
		status = false;
		printf("Failed to init GPS (Telementary)!\n");
	} else {
		printf("> GPS (Telementary) subsystem \tINIT OK\n");
	}

	// Notify the user if any subsystem initialization failed
	if (status == false) {
		printf("[!] Try typing 'init pigpiod' before running program!\n");
	}
	
	// Configure GPIO for steering motor enable
	set_mode(piHandle, steering_EN, PI_OUTPUT); // steering_EN = 19
	gpio_write(piHandle, steering_EN, 1); // steering_EN = 19

	// Return the overall initialization status
	return status;
}

// Disables all motors
void disable_motors(int pi) {
	// Set drive motor enable pins (REN and LEN) to LOW to stop power to the motor
	gpio_write(pi, drive_REN, 0);
	gpio_write(pi, drive_LEN, 0);
	
	// Disable steering motor
	// Set steering motor enable pin to LOW to cut power to the steering mechanism
	gpio_write(pi, steering_EN, 0);

	// Note: Motor control will remain disabled until explicitly re-enabled by other functions.
}

// Calculates a cruise control speed value (1-100) based on Lidar distance
int calculate_cruise(int cm_until_impact) {
    // Scale the cruise speed proportionally to the distance from the object
    // The speed decreases non-linearly as the vehicle gets closer to the object
    float new_speed = MAX_SPEED * (float(cm_until_impact) / (cm_buffer + MAX_SPEED));

    // Ensure the calculated speed does not exceed the maximum allowable speed
    if (new_speed > MAX_SPEED) {
        new_speed = MAX_SPEED;
    }

    // Prevent tailgating: if the vehicle is too close to an object, set the speed to 0
    if (cm_until_impact <= cm_buffer) {
        new_speed = 0;
    }

    // Log the calculated cruise value for debugging purposes
    cout << "Cruise Value: " << new_speed << " speed" << endl;

    // Return the cruise speed as an integer (rounded down automatically)
    return static_cast<int>(new_speed);
}

// Calculates the braking value (0 or 100) for cruise control based on Lidar distance
int calculate_cruise_breaks(int lidarDist) {
	// Check if cruise control is active
	if (!cruise_activated) {return 0;} // If cruise control is not active, no braking is needed

	// Determine if braking is required based on Lidar distance
	if (lidarDist <= cm_buffer + MAX_SPEED)	{
		// If the object is within a critical distance, apply full brakes
		cout << "\nYES BREAKS\n";
		return 100; // Full brake value
	}
	else {
		// If the object is at a safe distance, no braking is applied
		cout << "\nNO BREAKS\n";
		return 0; // No braking
	}
}

// Calculates a steering value (0-100) based on GPS data and destination coordinates
int calculate_autopilot_steering(double* gps_data, double* destination_data) {
	// Extract GPS data
	double current_lat = gps_data[0];			// Current latitude
	double current_lon = gps_data[1];			// Current longitude
	double current_course_angle = gps_data[2];		// Current course angle from GPS in degrees

	// Extract destination data
	double destination_lat = destination_data[0];	// Destination latitude
	double destination_lon = destination_data[1];	// Destination longitude


	// Note to Developer: The `gps_data` is being sent correctly, but the calculations for the course angle
	// (especially the bearing difference) is not producing the desired results. Further debugging is needed
	// to fix the math logic for accurate steering guidance.
	
	// Convert latitude and longitude from degrees to radians for trigonometric calculations
	double current_lat_rad = current_lat * PI / 180.0;
	double current_lon_rad = current_lon * PI / 180.0;
	double destination_lat_rad = destination_lat * PI / 180.0;
	double destination_lon_rad = destination_lon * PI / 180.0;

	// Calculate the difference in longitude between the current position and destination
	double delta_lon = destination_lon_rad - current_lon_rad;

	// Calculate the desired bearing angle (in radians) to the destination
	// This uses the haversine formula but may not be accurate
	double y = sin(delta_lon) * cos(destination_lat_rad);
	double x = cos(current_lat_rad) * sin(destination_lat_rad) - sin(current_lat_rad) * cos(destination_lat_rad) * cos(delta_lon);
	double desired_bearing_rad = atan2(y, x);

	// Convert bearing from radians to degrees
	double desired_bearing = fmod((desired_bearing_rad * 180.0 / PI) + 360.0, 360.0);

	// Convert the desired bearing from radians to degrees and normalize it to [0, 360]
	double bearing_diff = desired_bearing - current_course_angle;

	// Normalize the bearing difference to the range [-180, 180]
	if (bearing_diff > 180) {
		bearing_diff -= 360;
	} else if (bearing_diff < -180) {
		bearing_diff += 360;
	}

	// Use map function to get a smoother steering value
	// Map bearing_diff from [-180, 180] to [0, 100] for smoother control
	int steerValue = map(static_cast<long>(bearing_diff), -180, 180, 0, 100);

	// Debugging Information: Outputs the current GPS data, destination, and calculated values
	printf("GPS INFO:    Latitude: %.8f, Longitude: %.8f, Course: %.2f\n", current_lat, current_lon, current_course_angle);
	printf("DESTINATION: Latitude: %.8f, Longitude: %.8f\n", destination_lat, destination_lon);

	cout << "Desired Bearing: " << desired_bearing << "\n";
	cout << "Bearing Difference: " << bearing_diff << "\n";
	cout << "Mapped Steering Value: " << steerValue << "\n";

	// Return the calculated steering value
	return steerValue;
}

// Checks if cruise control is currently active
bool cruise_active() {
	// Returns the state of the cruise control system
	return cruise_activated;
}

// Checks if autopilot mode is currently active
bool autopilot_active() {
	// Returns the state of the autopilot system
	return autopilot_activated;
}


// Toggles the activation state of both autopilot and cruise control
void toggle_autopilot(bool toggle) {
	// Sets both cruise control and autopilot activation flags to the provided state
	cruise_activated = toggle;
	autopilot_activated = toggle;
	
	// Note: Cruise control and autopilot are toggled together for synchronized behavior.
}

// Manages the autopilot mode, including activation, GPS navigation, and destination handling
int run_autopilot(int toggle, double* gps_data, double* destination_data) {
	// --- Handle Autopilot Toggle ---
	if (autopilot_toggle == false && toggle == 6) {
		// Toggle autopilot mode on button press
		toggle_autopilot(autopilot_activated = !autopilot_activated); // Toggle the activation flag
		autopilot_toggle = true; // Set toggle flag to prevent repeated activation
	}
	else if(toggle == 0) {
		// Reset toggle flag when button is released
		autopilot_toggle = false;
	}

	// --- Exit if Autopilot is Deactivated ---
	if (!autopilot_activated) return 0; // Autopilot is not active, return no movement status

	// --- End of Course Check ---
	// If all destination data is -1, assume the end of the course is reached
	if (destination_data[0] == -1 && destination_data[1] == -1 && destination_data[2] == -1) {
		cruise_activated = false; // Deactivate cruise control
		cout << "AUTOPILOT FINISHED! \n";
		return 0; // Stay at the current point
	}

	// --- Destination Reached Check ---
	double threshold = 0.00005; // Threshold for proximity to destination (latitude/longitude difference)
	if (fabs(gps_data[0] - destination_data[0]) <= threshold && fabs(gps_data[1] - destination_data[1]) <= threshold) {
		// Current location is within the threshold of the destination
		cout << "Reached destination, moving to next point." << endl;
		return 1; // Move to the next destination
	}
	
	// --- Default State ---
	return 0; // Stay at the current point if destination is not reached
}