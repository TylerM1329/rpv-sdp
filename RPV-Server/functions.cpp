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

using namespace std;

int cm_buffer = 200;

const int drive_RPWM = 17;
const int drive_LPWM = 27;
const int drive_REN = 21;
const int drive_LEN = 20;

const int steering_RPWM = 24;
const int steering_LPWM = 23;
const int steering_EN = 19;

int cruise_activated = 0;
int cruise_toggle = 0;

int MAX_SPEED = 100;


long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int run_camera_pan(int direction) {
	static int currPos = 14;
	if (direction == 1) {	// pressing X (left)
		if (currPos < 25)
			currPos++;			// increment
	} else if (direction == 2) {
		if (currPos > 3)
			currPos--;			// decrement
	} else if (direction == 3) {
		currPos = 14;			// center
	}
	return currPos;
}

void run_acceleration(int pi, int intAccel, int gear, int lidarDist)
{
	if (intAccel)
	{
		intAccel = MAX_SPEED;
	}

	if (cruise_activated) { // adjust intAccel based on what is in front of car
		if (gear != 1) { // don't cruise control when in reverse gear
			// y = x * (1 - k / 800)
			intAccel = map(calculate_cruise(lidarDist) , 0, 100, 0, 255);
			gear = 0;
			cout << intAccel;
			cout << "\n";
		}
	}
	else if (intAccel)
	{
		intAccel = map(MAX_SPEED, 0, 100, 0, 255);
	}

	if (intAccel) {
		if (gear == 1) {						// REVERSE
			printf("Drive motor engaged\n");
			set_mode(pi, drive_REN, PI_OUTPUT);
			set_mode(pi, drive_LEN, PI_OUTPUT);
			gpio_write(pi, drive_REN, 1); 			// enable forward (both need to be high)
			gpio_write(pi, drive_LEN, 1); 			// enable backward (both need to be high)
			set_PWM_dutycycle(pi, drive_RPWM, 0); 			// set forward PWM 0
			set_PWM_dutycycle(pi, drive_LPWM, intAccel); 		// set backward PWM to intAccel
		} else if (gear == 0) {					// FORWARD
			printf("Drive motor engaged\n");
			set_mode(pi, drive_REN, PI_OUTPUT);
			set_mode(pi, drive_LEN, PI_OUTPUT);
			gpio_write(pi, drive_REN, 1); 			// enable forward (both need to be high)
			gpio_write(pi, drive_LEN, 1); 			// enable backward (both need to be high)
			set_PWM_dutycycle(pi, drive_RPWM, intAccel); 		// set forward PWM to intAccel
			set_PWM_dutycycle(pi, drive_LPWM, 0); 			// set backward PWM 0
		}
	} else {									// ALLOW COASTING
			printf("Drive motor coasting\n");			
			set_mode(pi, drive_REN, PI_OUTPUT);	
			set_mode(pi, drive_LEN, PI_OUTPUT);
			gpio_write(pi, drive_REN, 0); 			// disable forward
			gpio_write(pi, drive_LEN, 0); 			// disable backward
			set_PWM_dutycycle(pi, drive_RPWM, 0); 			// set forward PWM 0
			set_PWM_dutycycle(pi, drive_LPWM, 0); 			// set backward PWM 0
		}
}

int setup_TCP_Server(char ip[], int port)
{
	int clientSocket;
	//  Initiate the TCP configurations.
loop:
	//create socket
	
	int listening = socket(AF_INET, SOCK_STREAM, 0);

	if (listening == -1) //means there is an error in creating the socket
	{
		cerr << "Can't create socket";
		return -1;
	}

	//making a hint to bind socket to IP/port
	sockaddr_in hint;           //creates a hint to be binded to socket so client can connect to server
	hint.sin_family = AF_INET;  // Makes it IPv4
	hint.sin_port = htons(port);   //big endian or little endian conversion port
	inet_pton(AF_INET, ip, &hint.sin_addr);

	if (bind(listening, (sockaddr*)&hint, sizeof(hint)) == -1)    //checks if the hint binding was successful
	{
		cerr << "Can't bind to IP/Port";
		goto loop;
	}

	//mark the socket for listening
	if (listen(listening, SOMAXCONN) == -1)
	{
		cerr << "Can't listen";
		return -3;
	}

	//accept a call
	sockaddr_in client;     //creates socket address for client so they can connect
	socklen_t clientSize = sizeof(client);
	char host[NI_MAXHOST];
	char svc[NI_MAXSERV];

	clientSocket = accept(listening, (sockaddr*)&client, &clientSize);

	//close the listening port
	close(listening);
	memset(host, 0, NI_MAXHOST);
	memset(svc, 0, NI_MAXSERV);

	int result = getnameinfo((sockaddr*)&client, clientSize, host, NI_MAXHOST, svc, NI_MAXSERV, 0);

	if (result)
	{
		cout << host << " connected on " << svc << " connection 1"<<endl;
	}
	else
	{
		inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
		cout << host << " connected on " << ntohs(client.sin_port) << " connection 2"<<endl;
	}
	cout << "EXITING TCP STARTUP" << endl;
	return clientSocket;
}

void run_brake_lights(int brakeValue, int piHandle, int lightingHandle) {
    static bool brakeFlag1 = 1;
    static bool brakeFlag2 = 1;
    if (brakeValue > 50) {					// 60 is when brake actually starts to grab
		brakeFlag2 = 1;						// brake flags used to ensure I2C commands are issued only once
		if (brakeFlag1) {
			i2c_write_byte(piHandle, lightingHandle, 6);	
			brakeFlag1 = 0;
		}
	} else {
		brakeFlag1 = 1;
		if (brakeFlag2) {
			i2c_write_byte(piHandle, lightingHandle, 7);
			brakeFlag2 = 0;
		}
	}
}

void run_reverse_lights(int gear, int piHandle, int lightingHandle) {
	static bool gearFlag1 = 1;
	static bool gearFlag2 = 1;
	if (gear) {
			gearFlag2 = 1;
			if (gearFlag1) {
				//i2c_write_byte(piHandle, lightingHandle, 6); // change 6 to something else
				gearFlag1 = 0;
			}
		} else {
			gearFlag1 = 1;
			if (gearFlag2) {
				//i2c_write_byte(piHandle, lightingHandle, 7); // change 7 to something else
				gearFlag2 = 0;
			}
		}
}

void run_turn_signals(int buttons2, int piHandle, int lightingHandle) {
	cout << "Started running turn signal ";
	cout << buttons2;
	cout << endl;
	static bool turnFlag1 = 1;
	static bool turnFlag2 = 1;
	if (buttons2 == 1) {
		turnFlag2 = 1;
		if (turnFlag1) {
			i2c_write_byte(piHandle, lightingHandle, 2);	
			turnFlag1 = 0;
		}
	} else if (buttons2 == 2) {
		turnFlag1 = 1;
		if (turnFlag2) {
			i2c_write_byte(piHandle, lightingHandle, 4);
			turnFlag2 = 0;	
		}
	} else {	// default state, set flags back to initial state. 
		turnFlag1 = 1;
		turnFlag2 = 1;
	}
	cout << "Done running turn signal";
	cout << endl;
}

void run_cv_status_led(int status, int piHandle, int lightingHandle) {
	static bool flag1 = 1;
	static bool flag2 = 1;
	if (status == 1) {
		flag2 = 1;
		if (flag1) {
			i2c_write_byte(piHandle, lightingHandle, 8);	
			flag1 = 0;
		}
	} else if (status == 0) {
		flag1 = 1;
		if (flag2) {
			i2c_write_byte(piHandle, lightingHandle, 9);
			flag2 = 0;	
		}
	} else {	// default state, set flags back to initial state. 
		flag1 = 1;
		flag2 = 1;
	}
}

void run_headlights(int toggle, int piHandle, int lightingHandle) {
	//i2c_write_byte(piHandle, lightingHandle, 0); // 0 ON, 1 OFF
	static bool headFlag1 = 1;
	static bool headFlag2 = 1;
	static bool headlightEn = 0;
	if (toggle == 4) {
		headFlag2 = 1;
		if (headFlag1) {
			headlightEn = !headlightEn;
			if (headlightEn) {
				printf("ENABLE HEADLIGHT CMD SENT");
				i2c_write_byte(piHandle, lightingHandle, 0);
			} else {
				printf("DISABLE HEADLIGHT CMD SENT");
				i2c_write_byte(piHandle, lightingHandle, 1);
			}
			headFlag1 = 0;
		}
	} else {
		headFlag1 = 1;
		if (headFlag2) {
			headFlag2 = 0;
		}
	}
}

void run_steering(int deadband, int piHandle, int targetPos, int currentPos, int steeringDutyCycle) {
	if ((currentPos <= targetPos + deadband) && (currentPos >= targetPos - deadband)) {
			printf("Steering: CENTERED\n");
			set_PWM_dutycycle(piHandle, steering_RPWM, 0);
			set_PWM_dutycycle(piHandle, steering_LPWM, 0);
    	}

    	if (currentPos > targetPos + deadband) {
			printf("Steering: TURNING RIGHT\n");
			set_PWM_dutycycle(piHandle, steering_RPWM, steeringDutyCycle);
			set_PWM_dutycycle(piHandle, steering_LPWM, 0);
    	}

    	if (currentPos < targetPos - deadband) {
			printf("Steering: TURNING LEFT\n");
			set_PWM_dutycycle(piHandle, steering_RPWM, 0);
			set_PWM_dutycycle(piHandle, steering_LPWM, steeringDutyCycle);
    	}
}

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


int run_lidar(int toggle, int piHandle, int lidarHandle) {
	if (cruise_toggle == false && toggle == 5)
	{
		cruise_activated = !cruise_activated; // More elegant if-else statement for toggling cruise_control
		cruise_toggle = true;
	}
	else if(toggle == 0)
	{
		cruise_toggle = false;
	}

	uint16_t distance;
    char data[2];  // Array to store the 2 bytes of distance

    // Read 2 bytes from the lidar sensor
    int bytesRead = i2c_read_device(piHandle, lidarHandle, data, 2);
	  if (bytesRead == 2) {
		distance = (data[1] << 8) | data[0];
		cout << "Lidar Distance: " << distance << " cm" << std::endl;

		if (!cruise_activated) {return -1;}
		cout << "Cruise Value: " << calculate_cruise(distance) << " speed" << endl;
	  }
	  else
	  {
		cout << "Lidar failed. Reinitializing I2C handle...\n";
		i2c_close(piHandle, lidarHandle);
		lidarHandle = i2c_open(piHandle, 1, 0x11, 0);
	  }

    return distance;  // Return the distance if you need it
}

int run_GPS(int piHandle, int GPSHandle) {
	uint16_t Loca;
    char data[11];  // Array to store the 2 bytes of Loca

	int bytesRead = i2c_read_device(piHandle, GPSHandle, data, 11);
		if (bytesRead == 11) {
        // Process latitude (first 4 bytes)
        long lat = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
        cout << "Latitude: " << lat / 1000000.0 << std::endl;

        // Process longitude (next 4 bytes)
        long lon = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];
        cout << "Longitude: " << lon / 1000000.0 << std::endl;

        // Process course (next 2 bytes)
        int course = (data[9] << 8) | data[8];
        cout << "Course: " << course / 100.0 << " degrees" << std::endl;

        // Process control bit (last byte)
        int ctrl = data[10];
        cout << "Control: " << ctrl << std::endl;
    } else {
        cout << "Error: Expected 11 bytes, but got " << bytesRead << " bytes." << std::endl;
    }

    return 0;  // Return success
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

void get_user_options(int &serialOption, int &ASoption) {
	int userOption = 0;
userInput:
	cout << "\nUser Options: 1 for VRX/XBOX control only, 2 for VRX/XBOX + lane keep assist: ";
	cin >> userOption;
	switch (userOption)
	{
		case 1: {
			serialOption = 0;
			cout << "> Serial port DISABLED" << endl;
			break;
		}
		case 2: {
			serialOption = 1;
			cout << "> Serial port ENABLED" << endl;
			break;
		}
		default: {
			cout << "Invalid input!" << endl;
			goto userInput;
		}
	}
userInput2:
//-----------------------------------------------------------------------------------------------------------*
	// DEPRECIATED CODE. NO LONGER WORKS AND WILL STAND STILL IF SET TO 1
	// cout << "Active Safety: 1 for ON, 0 for OFF: ";
	// cin >> userOption;
	userOption = 2;
	switch (userOption)
	{
		case 2: {
			ASoption = 0;
			cout << "> Active Safety DEPRECIATED" << endl;
			break;
		}
		case 1: {
			ASoption = 1;
			cout << "> Active Safety ENABLED" << endl;
			break;
		}
		case 0: {
			ASoption = 0;
			cout << "> Active Safety DISABLED" << endl;
			break;
		}
		default: {
			cout << "Invalid input!" << endl;
			goto userInput2;
		}
	}
//-----------------------------------------------------------------------------------------------------------*
	cout << "\nSet max speed (1-100): ";
	cin >> userOption;

	while(userOption <= 0)
	{
		cout << "\nSet max speed (1-100): ";
		cin >> userOption;
	}

	if (userOption > 100) {userOption = 100;}
	MAX_SPEED = userOption;

	cout << "Speed set to ";
	cout << userOption << "\n";
}


void parse_control_data(int dataReady, char buffer[], int &gear, int &steerValue, int &accelValue, int &brakeValue, int &buttons1, int &buttons2, int &lidarDist, int &GPSLoca) {
	char *token;
	if (dataReady) {							// if data is ready (marked above) then proceed to parse
			// parse fwd/rvs
			token = strtok(buffer, "-");		// strtok() breaks string (buf) into a series of tokens using "-" delimiter
			gear = (token[0] == 'R');				// check if byte 0 of token contains R. R = 1, F = 0
			// parse steering
			token = strtok(NULL, "-");
			steerValue = atoi(token);				// convert string to int
			// parse acceleration
			token = strtok(NULL, "-");
			accelValue = atoi(token);
			if (accelValue > 0) { cruise_activated = false; }
			// parse braking
			token = strtok(NULL, "-");
			brakeValue = atoi(token);
			if (brakeValue > 0) { cruise_activated = false; }
			// parse button set 1
			token = strtok(NULL, "-");
			buttons1 = atoi(token);
			// parse button set 2
			token = strtok(NULL, "-");
			buttons2 = atoi(token);
		}
}

int get_cv_flag(int enable, char buffer[]) {	// HARDCODED INDEX IN BUFFER!
	static bool CVbounceFlag = 0;
	static bool CVenable = 0;
	static int CVbounceTimer = 0;
	if (buffer[16] == '3' && CVbounceFlag == 0) {			// if CV control button pressed and bounce flag zero
		CVbounceFlag = 1;										// set bounce flag to prevent re-execution of code below
		if (enable)
			CVenable = !CVenable;								// toggle CVenable if lane keeping has been selected by user. default 0.
	} else {													// if bounce flag is set
		CVbounceTimer++;										// increment timer
		//printf("timer = %d\n", CVbounceTimer);				// debug
		if (CVbounceTimer > 4) {								// if the timer has reached 2, reset the timer and the flag to allow CV control
			CVbounceTimer = 0;									// button to flip CVenable
			CVbounceFlag = 0;									
		}
	}
	return CVenable;
}


// zzz
int init_IO(int &piHandle, int &ultrasonicHandle, int ultrasonicAddr, int &lightingHandle, int lightingAddr, int &adcHandle, int &serialHandle, int &lidarHandle, int lidarAddr, int& GPSHandle, int GPSAddr) {
	bool status = 1;
	piHandle = pigpio_start(NULL, NULL);
	if (piHandle < 0) {
		status = 0;
		printf("[!] Failed to connect to pigpio daemon! Attempted to connect to localhost:8888\n");
	} else
        printf("> pigpio on localhost:8888 \tINIT OK\n");
	//----------------------------------------------------------------------------------------------------/
	ultrasonicHandle = i2c_open(piHandle, 1, ultrasonicAddr, 0);		// init I2C and assign handle to "ultrasonicHdl" 
	if (ultrasonicHandle < 0) {
		status = 0;
		printf("[!] Failed to init ultrasonic subsystem!\n");
	} else
		printf("> ultrasonic subsystem \t\tINIT OK\n");
	//----------------------------------------------------------------------------------------------------/
	lightingHandle = i2c_open(piHandle, 1, lightingAddr, 0);			// init I2C and assign handle to "lightingHdl"
	if (lightingHandle < 0) {
		status = 0;
		printf("[!] Failed to init lighting subsystem!\n");
	} else
		printf("> lighting subsystem \t\tINIT OK\n");
	//----------------------------------------------------------------------------------------------------/
	adcHandle = spi_open(piHandle, 0, 1000000, 0);						// init SPI and assign handle to "adc"
	if (adcHandle < 0) {
		status = 0;
		printf("[!] Failed to init ADC!\n");
	} else
		printf("> ADC \t\t\t\tINIT OK\n");
	//----------------------------------------------------------------------------------------------------/
	char port[] = "/dev/ttyS0";
	serialHandle = serial_open(piHandle, port, 115200, 0);			// init serial and assign handle to "serialHdl"
	if (serialHandle < 0) {
		status = 0;
		printf("[!] Failed to init serial! Try turning car off and on again!\n");
	} else
		printf("> serial open %s \tINIT OK\n", port);
	//----------------------------------------------------------------------------------------------------/

	lidarHandle = i2c_open(piHandle, 1, lidarAddr, 0);
	if (lidarHandle < 0) {
		status = 0;
		printf("[!] Failed to init lidar!\n");
	} else
		printf("> lidar subsystem \t\tINIT OK\n");

	//----------------------------------------------------------------------------------------------------/
	
	GPSHandle = i2c_open(piHandle, 1, GPSAddr, 0);
	if (GPSHandle < 0) {
		status = 0;
		printf("[!] Failed to init GPS (Telementary)!\n");
	} else
		printf("> GPS (Telementary) subsystem \t\tINIT OK\n");

	//----------------------------------------------------------------------------------------------------/
	set_mode(piHandle, steering_EN, PI_OUTPUT); // steering_EN = 19
	gpio_write(piHandle, steering_EN, 1); // steering_EN = 19
	return status;
}

void disable_motors(int pi) { // disable motors. gets overridden if other run functions execute
	// disable drive motor
	gpio_write(pi, drive_REN, 0);
	gpio_write(pi, drive_LEN, 0);
	// disable steering motor
	gpio_write(pi, steering_EN, 0);
	
}


// Returns a cruise value of 1-100, using accel and lidar info
int calculate_cruise(int cm_until_impact) {
	// Calculate the estimated new speed based on time to impact
	float new_speed = 0.25 * cm_until_impact - 0.25 * MAX_SPEED;

	// Ensure new_speed does not exceed max_speed
	if (new_speed > MAX_SPEED) {
    	new_speed = MAX_SPEED;
  	}
	// Prevents tailgaiting by cm_buffer amount
	if (cm_until_impact <= cm_buffer) {
		new_speed = 0;

		// No negative speed allowed
		if (new_speed < 0)
		new_speed = 0;
	}

	// Return the average speed
	return (new_speed);
}

int calculate_cruise_breaks(int lidarDist)
{
	if (lidarDist >= 0 && lidarDist <= 120 + MAX_SPEED)
	{
		return 100;
	}
	else
	{
		return 0;
	}
}
