/*
Version before adding selector between serial and socket.
*/


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

//#include <pigpio.h>
#include <pigpiod_if2.h>

#include <cstdlib>
#include <signal.h>

using namespace std;

void initialize_GPIO(void);
void run_camera_pan(int &currPos, int direction);
int setup_TCP_Server(string ip, int port);			// Set's up configuration for Control Capture and TCP Client Communication using 
void run_acceleration(int intAccel, int gear);				// Sends PWM control signals to DC Accel. Motor;
void run_braking(int intBrake, string &strBrake);	// Sends EN1 and EN2 signals to Braking Actuator
long map(long x, long in_min, long in_max, long out_min, long out_max);
void signal_callback_handler(int signum);
void run_steering(int deadband, int targetPos, int currentPos, int steeringDutyCycle);

int clientSocket;
// PIN DEFINITIONS
const int steering_RPWM = 24;
const int steering_LPWM = 23;
const int steering_EN = 19;

const int drive_RPWM = 17;
const int drive_LPWM = 27;
const int drive_REN = 21;
const int drive_LEN = 20;

const int camera_servo = 18;
const int brake_servo = 13;

const int ultrasonicAddr = 3;			// I2C address for ultrasonic subsystem
const int lightingAddr = 6;				// I2C address for lighting subsystem

int serialPort;
int pi = 0;								// var to hold pigpiod handle
int ultrasonicHdl = 0;					// var to hold ultrasonic subsystem handle
int lightingHdl = 0;					// var to hold lighting subsystem handle
int adc = 0;							// var to hold SPI ADC handle
int serialHdl = 0;						// var to hold serial handle

int timer = 0;

int main()
{
	// Register signal and signal handler
   	
	string ip;
	int port, source, userOption;
	bool serialRXenable;
	bool CVenable = 0;							// var to hold state of (computer vision) lane keeping option.
	int RXbyteCount = 0;						// byte count of received message
	int serialRXbyteCount = 0;
	char socketBuf[512];						// receive buffer for TCP socket
	char preParseBuf[18];						// buffer that enters parsing logic
	char serialBuf[13];							// receive buffer for serial port
	int serialAvail = 0;						// var to hold available bytes to be read
	int steerValue = 0;							// var to hold parsed steering value
	int accelValue = 0;							// var to hold parsed acceleration value
	int brakeValue = 0;							// var to hold parsed braking value
	int buttons1 = 0;							// var to hold first set of buttons (X and B). used for camera panning. these get held down
	int buttons2 = 0;							// var to hold left and right buttons. used for turn signals. brake/reverse handled internally. future: headlight control. CV toggle
	int gear = 0;								// flag to hold gear. 0 = fwd, 1 = rvs.
	int currPanPos = 14;						// var to hold current pan position. default 14 (center)
	char *token;								// token pointer used in parsing buf
	char spi_tx[3] = {1, 128, 0};  				// command to read CH0 from MCP3008
	char spi_rx[3];								// array to hold received SPI data
	int adc_data = 0;							// var to hold adc value
	int currentPos = 0;							// var to hold current position of steering
	int targetPos = 270;						// var to hold target steering position. default 270 (center-ish)
	int deadband = 15;							// var to hold deadband value for steering logic
	int steeringDutyCycle = 180;				// 0~255 PWM DC for steering. too high of a value will cause 
												// steering to oscillate around setpoint. 180 good full charge
	bool brakeFlag1 = 1;						// flag used to control sending I2C commands for braking
	bool brakeFlag2 = 1;						// flag used to control sending I2C commands for un-braking

	initialize_GPIO();							// init GPIO, I2C, SPI

	signal(SIGINT, signal_callback_handler);	// make sure to call this after calling gpioInitialize()
    
	// Get user input on what type of connection is being used.
	connectionInput:
	cout << "What address should I listen on? 1 for local, 2 for VPN, 3 for custom: ";
	cin >> source;
	switch (source)
	{
		case 1: {
			ip = "192.168.5.132";
			port = 54000;
			break;
		}
		case 2: {
			ip = "10.8.0.2";
			port = 54000;
			break;
		}
		case 0: {
			cout << "Enter IP: ";
			cin >> ip;
			cout << "\nEnter Port: ";
			cin >> port;
			break;
		}
		default: {
			cout << "Invalid input!" << endl;
			goto connectionInput;
		}
	}
	userInput:
	cout << "\nUser Options: 1 for VRX/XBOX control only, 2 for VRX/XBOX + lane keep assist: ";
	cin >> userOption;
	switch (userOption)
	{
		case 1: {
			serialRXenable = 0;
			cout << "Serial port DISABLED" << endl;
			break;
		}
		case 2: {
			serialRXenable = 1;
			cout << "Serial port ENABLED" << endl;
			break;
		}
		default: {
			cout << "Invalid input!" << endl;
			goto userInput;
		}
	}

	cout << "Opening Connection. Listening at " << ip << ":" << port << endl;

loop:
	// Call TCP server setup function
	int success;
	success = setup_TCP_Server(ip, port);
	if (success ==! 0)
		return success;
    cout << "\nConnected" << endl;

	// Main loop
	while(true) // Change this to while(connected to client).
    {
		// RECEIVE AND PARSE SERIAL DATA
		if (serialRXenable) {
			serialAvail = serial_data_available(pi, serialHdl);
			printf("Serial available: %d\n", serialAvail);
			if (serialAvail > 0)
				serial_read(pi, serialHdl, serialBuf, sizeof(serialBuf));

			printf("RX: %s\n", serialBuf);
		}
		
		memset(socketBuf, 0, sizeof(socketBuf));													// clear receive buffer
        RXbyteCount = recv(clientSocket, socketBuf, sizeof(socketBuf), 0);
		if (RXbyteCount == 0) {
			cerr << "There was a connection issue. Waiting for reconnection..." << endl;
			goto loop;
		}
		printf("recv()'d %d bytes of data in buf\n", RXbyteCount);
		printf("Received: %s\n", socketBuf);

		// two possible sources: serialBuf, socketBuf

		// SOCKET DATA PARSING LOGIC
		if (RXbyteCount > 0) {						// only parse if the buffer actually has something
			// parse fwd/rvs
			token = strtok(socketBuf, "-");			// strtok() breaks string (buf) into a series of tokens using "-" delimiter
			gear = (token[0] == 'R');				// check if byte 0 of token contains R. R = 1, F = 0

			printf("Gear: %d\n", gear);

			// parse steering ----------------------------------------------------------/
			token = strtok(NULL, "-");
			steerValue = atoi(token);				// convert string to int
			//int pot = map(analogRead(101), 0, 50, 0, 15);
			//printf("Buffer: %d\n", pot);
			printf("Steering: %d\n", steerValue);

			// parse acceleration ----------------------------------------------------------/
			token = strtok(NULL, "-");
			accelValue = atoi(token);
			printf("Acceleration: %d\n", accelValue);

			// parse braking ----------------------------------------------------------/
			token = strtok(NULL, "-");
			brakeValue = atoi(token);
			printf("Braking: %d\n", brakeValue);

			// parse button set 1 ----------------------------------------------------------/
			token = strtok(NULL, "-");
			buttons1 = atoi(token);
			printf("Button set 1: %d\n", buttons1);
			
			// parse button set 2 ----------------------------------------------------------/
			token = strtok(NULL, "-");
			buttons2 = atoi(token);
			printf("Button set 2: %d\n", buttons2);
			
		}
		// DO MAIN FUNCTIONS HERE
		// CONTROL ACCELERATION, STEERING, SERVOS, AND SAMPLE ADC
		run_acceleration(map(accelValue, 0, 100, 0, 255), gear);
		run_camera_pan(currPanPos, buttons1);
		printf("Pan position: %d\n", currPanPos);
		set_servo_pulsewidth(pi, camera_servo, map(currPanPos, 3, 25, 600, 2400));
		set_servo_pulsewidth(pi, brake_servo, map(brakeValue, 0, 100, 2050, 2220)); // relaxed | braking
		spi_xfer(pi, adc, spi_tx, spi_rx, 3);
		adc_data = (spi_rx[1] << 8) | spi_rx[2] & 0x3FF;
		run_steering(deadband, map(steerValue, 0, 100, 105, 380), adc_data, steeringDutyCycle);

		printf("Actual steering position: %d\n", adc_data);
		// CONTROL TURN SIGNALS
		if (buttons2 > 0) {
			if (buttons2 == 1)
				i2c_write_byte(pi, lightingHdl, 4);		// will fire continuously if buttons are held down. for turn signals
			if (buttons2 == 2)
				i2c_write_byte(pi, lightingHdl, 5);		// will fire continuously if buttons are held down. for turn signals
			if (buttons2 == 3)
				if (serialRXenable)
					CVenable = !CVenable;				// toggle CVenable if lane keeping has been selected by user. default 0.
		}
		printf("CVenable: %d\n", CVenable);
		// CONTROL BRAKE LIGHTS
		if (brakeValue > 50) {					// 60 is when brake actually starts to grab
			brakeFlag2 = 1;						// brake flags used to ensure I2C commands are issued only once
			if (brakeFlag1) {
				i2c_write_byte(pi, lightingHdl, 6);	
				brakeFlag1 = 0;
			}
		} else {
			brakeFlag1 = 1;
			if (brakeFlag2) {
				i2c_write_byte(pi, lightingHdl, 7);
				brakeFlag2 = 0;
			}
		}
		

		printf("\n");
    }

    
    return 0;
}

void initialize_GPIO(void)
{
	pi = pigpio_start(NULL, NULL);
	if(pi < 0)
        printf("> Failed to connect to pigpio daemon! Attempted to connect to localhost:8888\n");
    else
        printf("> pigpio on localhost:8888 \tINIT OK\n");
	//----------------------------------------------------------------------------------------------------/
	ultrasonicHdl = i2c_open(pi, 1, ultrasonicAddr, 0);		// init I2C and assign handle to "ultrasonicHdl" 
	if(ultrasonicHdl < 0)
		printf("> Failed to init ultrasonic subsystem!\n");
	else
		printf("> ultrasonic subsystem \t\tINIT OK\n");
	//----------------------------------------------------------------------------------------------------/
	lightingHdl = i2c_open(pi, 1, lightingAddr, 0);			// init I2C and assign handle to "lightingHdl"
	if(lightingHdl < 0)
		printf("> Failed to init lighting subsystem!\n");
	else
		printf("> lighting subsystem \t\tINIT OK\n");
	//----------------------------------------------------------------------------------------------------/
	adc = spi_open(pi, 0, 1000000, 0);						// init SPI and assign handle to "adc"
	if(adc < 0)
		printf("> Failed to init ADC!\n");
	else
		printf("> ADC \t\t\t\tINIT OK\n");
	//----------------------------------------------------------------------------------------------------/
	char port[] = "/dev/ttyS0";
	serialHdl = serial_open(pi, port, 115200, 0);			// init serial and assign handle to "serialHdl"
	if(serialHdl < 0)
		printf("> Failed to init serial!\n");
	else
		printf("> serial open %s \tINIT OK\n", port);
	//----------------------------------------------------------------------------------------------------/
	set_mode(pi, steering_EN, PI_OUTPUT);
	gpio_write(pi, steering_EN, 1); 
	//set_mode(pi, steering_RPWM, PI_OUTPUT);
	//set_mode(pi, steering_LPWM, PI_OUTPUT);

	

}

int setup_TCP_Server(string ip, int port)
{
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
	inet_pton(AF_INET, ip.c_str(), &hint.sin_addr);

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

	if (clientSocket == -1)
	{
		cerr << "Problem with client connection";
		return -4;
	}

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
	cout << "EXITING TCP STARTUP" <<endl;
	return 0;
}

void run_acceleration(int intAccel, int gear)
{
	int localAccel;
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
			set_mode(pi, drive_REN, PI_INPUT);
			set_mode(pi, drive_LEN, PI_INPUT);
			gpio_write(pi, drive_REN, 0); 			// disable forward
			gpio_write(pi, drive_LEN, 0); 			// disable backward
			set_PWM_dutycycle(pi, drive_RPWM, 0); 			// set forward PWM 0
			set_PWM_dutycycle(pi, drive_LPWM, 0); 			// set backward PWM 0
		}
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void run_camera_pan(int &currPos, int direction) {
	if (direction == 1) {	// pressing X (left)
		if (currPos < 25)
			currPos++;			// increment
	} else if (direction == 2) {
		if (currPos > 3)
			currPos--;			// decrement
	} else if (direction == 3) {
		currPos = 14;			// center
	}
}

void run_steering(int deadband, int targetPos, int currentPos, int steeringDutyCycle) {
	if ((currentPos <= targetPos + deadband) && (currentPos >= targetPos - deadband)) {
      		//gpio_write(pi, steering_RPWM, 0); 
			//gpio_write(pi, steering_LPWM, 0); 
			set_PWM_dutycycle(pi, steering_RPWM, 0);
			set_PWM_dutycycle(pi, steering_LPWM, 0);
    	}

    	if (currentPos > targetPos + deadband) {
      		//gpio_write(pi, steering_RPWM, 1); 
			//gpio_write(pi, steering_LPWM, 0); 
			set_PWM_dutycycle(pi, steering_RPWM, steeringDutyCycle);
			set_PWM_dutycycle(pi, steering_LPWM, 0);
    	}

    	if (currentPos < targetPos - deadband) {
      		//gpio_write(pi, steering_RPWM, 0); 
			//gpio_write(pi, steering_LPWM, 1); 
			set_PWM_dutycycle(pi, steering_RPWM, 0);
			set_PWM_dutycycle(pi, steering_LPWM, steeringDutyCycle);
    	}
}

void signal_callback_handler(int signum) {
   cout << "CTRL+C caught! Terminating..." << signum << endl;
   //serClose(serialPort);
   spi_close(pi, adc);
   pigpio_stop(pi);
   close(clientSocket);
   
   // Terminate program
   exit(signum);
}
