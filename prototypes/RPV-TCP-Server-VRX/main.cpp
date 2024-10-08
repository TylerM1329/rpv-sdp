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

// Pi's GPIO Configuration
#include <wiringPi.h>	// For control signals sent to motor drivers
#include <softPwm.h>	// For PWM signal generation (sub-library of wiring pi)
#include <mcp3004.h>	// The ADC Configuration header

// Serial Library
#include <wiringSerial.h>
#include <errno.h>

using namespace std;

void initialize_PI(void);
void run_centering(void);
int init_serial(int);
int setup_TCP_Client(string ip, int port);			// Set's up configuration for Control Capture and TCP Client Communication using 
void run_acceleration(int intAccel, int gear);				// Sends PWM control signals to DC Accel. Motor;
void run_steering(int intSteer);					// Sends EN1 and EN2 signals to Steering Actuator 
void run_braking(int intBrake, string &strBrake);	// Sends EN1 and EN2 signals to Braking Actuator
void clear_digital_outputs(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);

int clientSocket, flag = 0;

const char steering_RPWM = 24;
const char steering_LPWM = 23;
const char steering_EN = 13;

const char drive_RPWM = 17;
const char drive_LPWM = 18;
const char drive_REN = 21;
const char drive_LEN = 20;
const char camera_servo = 27;

const char testPin = 26;

const int dataSize = 32;


int main()
{
	string ip;
	int flag = 0, port, source;
	int serialPort = init_serial(9600);			// initialize serial and store file descriptor
	int RXbyteCount = 0;						// byte count of received message
	char socketBuf[512];								// receive buffer from TCP socket
	int steerValue = 0;							// var to hold parsed steering value
	int accelValue = 0;							// var to hold parsed acceleration value
	int brakeValue = 0;							// var to hold parsed braking value
	int gear = 0;								// flag to hold gear. 0 = fwd, 1 = rvs.
	char *token;								// token pointer used in parsing buf
	
	initialize_PI();
	clear_digital_outputs();
    
	// Get user input on what type of connection is being used.
	input:
	cout << "What address should I listen on? 1 for local, 2 for VPN, 3 for custom: ";
	cin >> source;
	switch (source)
	{
		case 1:{
			ip = "192.168.8.132";
			port = 54000;
			break;
		}
		case 2:{
			ip = "10.8.0.2";
			port = 54000;
			break;
		}
		case 0:{
			cout << "Enter IP: ";
			cin >> ip;
			cout << "\nEnter Port: ";
			cin >> port;
			break;
		}
		default: {
			cout << "Not Valid.\n";
			goto input;
		}
	}
	cout << "\n";
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
		memset(socketBuf, 0, sizeof(socketBuf));													// clear receive buffer
        RXbyteCount = recv(clientSocket, socketBuf, sizeof(socketBuf), 0);
		if (RXbyteCount == 0) {
			cerr << "There was a connection issue. Waiting for reconnection..." << endl;
			goto loop;
		}
		printf("recv()'d %d bytes of data in buf\n", RXbyteCount);
		printf("Received: %s\n", socketBuf);
		// BEGIN PARSE LOGIC
		if (RXbyteCount > 0) {						// only parse if the buffer actually has something
			// parse fwd/rvs
			token = strtok(socketBuf, "-");				// strtok() breaks string (buf) into a series of tokens using "-" delimiter
			gear = (token[0] == 'R');				// check if byte 0 of token contains R. R = 1, F = 0

			printf("Gear: %d\n", gear);

			// parse steering ----------------------------------------------------------/
			token = strtok(NULL, "-");
			steerValue = atoi(token);				// convert string to int
				run_steering(steerValue);
			printf("Steering: %d\n", steerValue);

			// parse acceleration ----------------------------------------------------------/
			token = strtok(NULL, "-");
			accelValue = atoi(token);
				run_acceleration(accelValue, gear);
			printf("Acceleration: %d\n", accelValue);

			// parse braking ----------------------------------------------------------/
			token = strtok(NULL, "-");
			brakeValue = atoi(token);
			softPwmWrite(camera_servo, map(brakeValue, 0, 100, 0, 30));
			printf("Braking: %d\n", brakeValue);
			
		}
		


    }

    //close socket
    close(clientSocket);
    
    clear_digital_outputs();
    
    return 0;
}

void initialize_PI(void)
{
	//Wiring PI Setup
	if (wiringPiSetupGpio() == -1)
		cout << "Error" << endl;
	
	//pinMode(x, OUTPUT);			//brakes-reverse
	//pinMode(x, OUTPUT);			//brakes-forward
	//pinMode(25, OUTPUT); 			//brake enable
	
	pinMode(steering_RPWM, OUTPUT);		//Steering-reverse
	pinMode(steering_LPWM, OUTPUT);		//Steering-forward
	pinMode(steering_EN, OUTPUT); 		//steer enable
	
	softPwmCreate(drive_RPWM, 0, 100);	//acceleration
	softPwmCreate(drive_LPWM, 0, 100);	//acceleration
	pinMode(drive_REN, OUTPUT); 		//backward enable
	pinMode(drive_LEN, OUTPUT); 		//forward enable

	softPwmCreate(camera_servo, 0, 100);
	
	//ADC Setup
	mcp3004Setup(100, 0);
	
	clear_digital_outputs();
}

int setup_TCP_Client(string ip, int port)
{
	//  Initiate the TCP configurations.
loop:
	//create socket
	
	int listening = socket(AF_INET, SOCK_STREAM, 0);

	while (flag)
	{
		cout << "Listening..." << endl;
		if (listening != -1) flag = 0;
	}

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
	if (gear == 1) {							// REVERSE
		digitalWrite(drive_REN, 1); 			// enable forward (both need to be high)
		digitalWrite(drive_LEN, 1); 			// enable backward (both need to be high)
		softPwmWrite(drive_RPWM, 0); 			// set forward PWM 0
		softPwmWrite(drive_LPWM, intAccel); 	// set backward PWM to intAccel
		
	} else if (gear == 0) {						// FORWARD
		digitalWrite(drive_REN, 1); 			// enable forward (both need to be high)
		digitalWrite(drive_LEN, 1); 			// enable backward (both need to be high)
		softPwmWrite(drive_RPWM, intAccel); 	// set forward PWM to intAccel
		softPwmWrite(drive_LPWM, 0); 			// set backward PWM 0

	} else {									// STOPPED (if gear does not match any case)
		digitalWrite(drive_REN, 0); // disable forward
		digitalWrite(drive_LEN, 0); // disable backward
		softPwmWrite(drive_RPWM, 0); // set forward PWM 0
		softPwmWrite(drive_LPWM, 0); // set backward PWM 0
		//cout << "In deadzone";
	}
}

void run_steering(int intSteer) 
{
	int currPosition = analogRead(100);
	if (intSteer > 50) {
		cout << "In left zone" << endl;
		digitalWrite(steering_EN, 1);
		digitalWrite(steering_RPWM, 0);
		digitalWrite(steering_LPWM, 1);
	} else if (intSteer < 50) {
		cout << "In right zone" << endl;
		digitalWrite(steering_EN, 1);
		digitalWrite(steering_RPWM, 1);
		digitalWrite(steering_LPWM, 0);
	} else {
		cout << "In dead zone (center)" << endl;
		digitalWrite(steering_RPWM, 0);
		digitalWrite(steering_LPWM, 0);
		digitalWrite(steering_EN, 0);
	}
}

void clear_digital_outputs(void)
{
	digitalWrite(steering_EN, 0);
	digitalWrite(drive_REN, 0);
	digitalWrite(drive_LEN, 0);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void run_centering(void) {
	
}

int init_serial(int baud) {
	/*
	This function uses serialOpen(char *device, int baud) provided by WiringSerial.h.
	(From wiringpi.com)
	This [serialOpen()] opens and initialises the serial device and sets the baud rate. 
	It sets the port into “raw” mode (character at a time and no translations), 
	and sets the read timeout to 10 seconds. The return value is the file descriptor 
	or -1 for any error, in which case errno will be set as appropriate.
	*/
	int serial_port;
	if ((serial_port = serialOpen ("/dev/ttyS0", baud)) < 0) {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno));
  	}
	return serial_port;
}