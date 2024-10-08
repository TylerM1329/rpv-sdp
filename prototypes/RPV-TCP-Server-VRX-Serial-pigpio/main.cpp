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

#include <pigpio.h>

#include <cstdlib>
#include <signal.h>

using namespace std;

void initialize_GPIO(void);
void run_camera_pan(int &currPos, int direction);
int setup_TCP_Client(string ip, int port);			// Set's up configuration for Control Capture and TCP Client Communication using 
void run_acceleration(int intAccel, int gear);				// Sends PWM control signals to DC Accel. Motor;
void run_braking(int intBrake, string &strBrake);	// Sends EN1 and EN2 signals to Braking Actuator
long map(long x, long in_min, long in_max, long out_min, long out_max);
void signal_callback_handler(int signum);

int clientSocket;

const char steering_RPWM = 24;
const char steering_LPWM = 23;
const char steering_EN = 19;

const char drive_RPWM = 17;
const char drive_LPWM = 27;
const char drive_REN = 21;
const char drive_LEN = 20;

const char camera_servo = 18;
const char brake_servo = 13;

const char testPin = 26;

const int dataSize = 32;

int serialPort;

int main()
{
	// Register signal and signal handler
   	
	string ip;
	int port, source, userOption;
	bool serialRXenable;
	int RXbyteCount = 0;						// byte count of received message
	int serialRXbyteCount = 0;
	char socketBuf[512];						// receive buffer for TCP socket
	char serialBuf[13];							// receive buffer for serial port
	int steerValue = 0;							// var to hold parsed steering value
	int accelValue = 0;							// var to hold parsed acceleration value
	int brakeValue = 0;							// var to hold parsed braking value
	int buttons1 = 0;							// var to hold first set of buttons (X and B)
	int gear = 0;								// flag to hold gear. 0 = fwd, 1 = rvs.
	int currPanPos = 14;						// var to hold current pan position. default 14 (center)
	char *token;								// token pointer used in parsing buf
	
	initialize_GPIO();

	int handle = i2cOpen(1, 8, 0);

	signal(SIGINT, signal_callback_handler);	// make sure to call this after calling gpioInitialise()
    
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
			cout << "Serial port disabled" << endl;
			break;
		}
		case 2: {
			serialRXenable = 1;
			if ((serialPort = serOpen("/dev/ttyS0", 115200, 0)) < 0)
				cout << "Failed to open serial port!" << endl;
			else
				cout << "Listening for serial commands at /dev/ttyS0 at 115200 baud" << endl;
			break;
		}
		default: {
			cout << "Invalid input!" << endl;
			goto userInput;
		}
	}

	cout << "Opening Connection. Listening at " << ip << ":" << port << endl;

loop:
	// Call TCP client setup function
	int success;
	success = setup_TCP_Client(ip, port);
	if (success ==! 0)
		return success;
    cout << "\nConnected" << endl;

	// Main loop
	while(true) // Change this to while(connected to client).
    {
		// SERIAL PORT PARSING LOGIC											// add logic to only run this if serial is selected by user!!!
		if (serDataAvailable(serialPort > 0)) {
			serialRXbyteCount = serRead(serialPort, serialBuf, 13);
		}
		printf("Serial RX %d bytes: %s\n", serialRXbyteCount, serialBuf);
		
		memset(socketBuf, 0, sizeof(socketBuf));													// clear receive buffer
        RXbyteCount = recv(clientSocket, socketBuf, sizeof(socketBuf), 0);
		if (RXbyteCount == 0) {
			cerr << "There was a connection issue. Waiting for reconnection..." << endl;
			goto loop;
		}
		printf("recv()'d %d bytes of data in buf\n", RXbyteCount);
		printf("Received: %s\n", socketBuf);
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
			
		}
		// DO MAIN FUNCTIONS HERE
		run_acceleration(map(accelValue, 0, 100, 0, 255), gear);
		run_camera_pan(currPanPos, buttons1);
		printf("Pan position: %d\n", currPanPos);
		gpioServo(camera_servo, map(currPanPos, 3, 25, 600, 2400));
		gpioServo(brake_servo, map(brakeValue, 0, 100, 1500, 2000));
		i2cWriteByte(handle, steerValue);

		

		printf("\n");
    }

    
    return 0;
}

void initialize_GPIO(void)
{
	if(gpioInitialise() < 0)
        printf("Failed to initialize pigpio\n");
    else
        printf("pigpio INIT OK\n");
}

int setup_TCP_Client(string ip, int port)
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
			gpioSetMode(drive_REN, PI_OUTPUT);
			gpioSetMode(drive_LEN, PI_OUTPUT);
			gpioWrite(drive_REN, 1); 			// enable forward (both need to be high)
			gpioWrite(drive_LEN, 1); 			// enable backward (both need to be high)
			gpioPWM(drive_RPWM, 0); 			// set forward PWM 0
			gpioPWM(drive_LPWM, intAccel); 		// set backward PWM to intAccel
		} else if (gear == 0) {					// FORWARD
			printf("Drive motor engaged\n");
			gpioSetMode(drive_REN, PI_OUTPUT);
			gpioSetMode(drive_LEN, PI_OUTPUT);
			gpioWrite(drive_REN, 1); 			// enable forward (both need to be high)
			gpioWrite(drive_LEN, 1); 			// enable backward (both need to be high)
			gpioPWM(drive_RPWM, intAccel); 		// set forward PWM to intAccel
			gpioPWM(drive_LPWM, 0); 			// set backward PWM 0
		}
	} else {									// ALLOW COASTING
			printf("Drive motor coasting\n");			
			gpioSetMode(drive_REN, PI_INPUT);
			gpioSetMode(drive_LEN, PI_INPUT);
			gpioWrite(drive_REN, 0); 			// disable forward
			gpioWrite(drive_LEN, 0); 			// disable backward
			gpioPWM(drive_RPWM, 0); 			// set forward PWM 0
			gpioPWM(drive_LPWM, 0); 			// set backward PWM 0
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

void signal_callback_handler(int signum) {
   cout << "CTRL+C caught! Terminating..." << signum << endl;
   serClose(serialPort);
   gpioTerminate();
   close(clientSocket);
   
   // Terminate program
   exit(signum);
}