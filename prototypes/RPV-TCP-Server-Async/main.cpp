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

//Pi's GPIO Configuration
#include <wiringPi.h>	// For control signals sent to motor drivers
#include <softPwm.h>	// For PWM signal generation (sub-library of wiring pi)
#include <mcp3004.h>	// The ADC Configuration header

using namespace std;

void initialize_PI(void);
int setup_TCP_Client(string ip, int port);			// Set's up configuration for Control Capture and TCP Client Communication using 
void run_acceleration(int intAccel);				// Sends PWM control signals to DC Accel. Motor;
void run_steering(int intSteer);					// Sends EN1 and EN2 signals to Steering Actuator 
void run_braking(int intBrake, string &strBrake);	// Sends EN1 and EN2 signals to Braking Actuator
void clear_digital_outputs(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);

int clientSocket, flag = 0;

const char steering_RPWM = 24;
const char steering_LPWM = 23;
const char steering_EN = 13;

const char drive_PWM = 17;
const char drive_REN = 21;
const char drive_LEN = 20;


int main()
{
	string ip;
	int flag = 0, port, source;
	
	initialize_PI();
    
	// Get user input on what the internet source is.
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
	cout << "Opening Connection. Listening at " << ip << ":" << port;
	
loop:
	//Initalize TCP Client Configuration
	
	int success;
	success = setup_TCP_Client(ip, port);
	if (success ==! 0)			// If setup fails it returns the error number. 
		return success;
								// If success then continues down

// ALL CONTROL STUFF
    //while receiving- display message, echo message
    char steer[4096], accel[4096], brake[4096];
    
    int steeringBytes, brakingBytes, accelerationBytes;
    
    int intSteer, intBrake, intAccel;
    
    string strBrake;
    
    cout << "\nConnected" << endl;
    
	while(true) // Change this to while(connected to client).
    {
        //clear buffer
        memset(steer, 0, 4096);
        memset(accel, 0 , 4096);
        memset(brake, 0 , 4096);
        
    //////////////////////////////////////////////////////////
	//wait for a Steering message
	//////////////////////////////////////////////////////////
    steeringBytes = recv(clientSocket, steer, 4096, 0);
        
	if(steeringBytes == -1)
	{ 
            flag <<= 1;
            cerr << "There was a Connection issue" << endl;
            goto loop;
        }
	stringstream geek1(string(steer, 0, steeringBytes));
		
	geek1 >> intSteer;
	//cout << "Received steering: " << intSteer << endl;
		
	// Echo Steering Messsage
	//send(clientSocket, steer, steeringBytes + 1, 0);
		
	//////////////////////////////////////////////////////////
	//wait for braking message
	//////////////////////////////////////////////////////////
    brakingBytes = recv(clientSocket, brake, 4096, 0);
    if(steeringBytes == -1)
	{ 
            flag <<= 1;
            cerr<<"There was a Connection issue"<<endl;
            goto loop;
    }
    stringstream geek2(string(brake, 0, brakingBytes));
        
    geek2 >> intBrake;
    //cout << "Received braking " << intBrake << endl;
        
    //Echo Braking message
    //send(clientSocket, strBrake.c_str(), brakingBytes+1, 0);
        
	//////////////////////////////////////////////////////////
	//wait for acceleration message
	//////////////////////////////////////////////////////////
	accelerationBytes = recv(clientSocket, accel, 4096, 0);
	if (accelerationBytes == -1)
	{
		flag <<= 1;
		cerr << "There was a Connection issue" << endl;
		goto loop;
	}
	stringstream geek3(string(accel, 0, accelerationBytes));
		
	geek3 >> intAccel;
	//cout << "Received acceleration: " << intAccel << endl;
		
	//Echo Acceleration message
	//send(clientSocket, to_string(intAccel).c_str(), accelerationBytes + 1, 0);
		
	//////////////////////////////////////////////////////////
	// Now Output Control Signals through Pi's GPIO pins
	// Function to output control signals to steering actuator
	
	//run_steering(intSteer);
        
	// Function to output control signals to braking actuator
	
	//run_braking(intBrake, strBrake);
        
	// Function to output control signals to kart's DC Motor for acceleration
	//cout << "\nChecking brake val: " << intBrake << endl;
	//if(intBrake <= 24000){
		run_acceleration(intAccel);//}
        
        // Prints to console what control parameters are captured and being set over by the simulator/controller
        cout << "Steering: " << intSteer << " Braking:  "  << intBrake << " Accelerate: " << intAccel << endl;
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
	
	softPwmCreate(drive_PWM, 0, 100);	//acceleration
	pinMode(drive_REN, OUTPUT); 		//backward enable
	pinMode(drive_LEN, OUTPUT); 		//forward enable
	
	
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

void run_steering(int intSteer)
{
	//Math to translate intSteer and run the steering actuator
	int position, value;
	value = (((intSteer-(65535/2)) * 280) / 65535) + 565;	// "Value" = control that is captured and sent over. "Position" = current actuator position
	value = (value / 3) - 30;							// This function's purpose is to move the actuator's real-time "Position" to match the desired control "Value"
	// value is subtracted by 30 to center the wheels. Can be adjusted should the center need be readjusted
	position = analogRead(100);
	position = position / 3;
	cout << "\nPosition = " << position << " value = " << value << endl;
	if((value > (position + 2)) || (value < (position - 2) ) )// checks if value is outside of bounds of position +/- 1 (value can be adjusted later on)  
	{
		
		cout << "\nSTEERING" << endl;
		// Turn Right
		if (position < value)	
		{
			digitalWrite(steering_EN, 1); // Enable Steering
			digitalWrite(steering_RPWM, 0);
			digitalWrite(steering_LPWM, 1);
			position = (analogRead(100)) / 3;		// Re-check position.
		}

		// Turn Left
		else if (position > value)	
		{
			digitalWrite(steering_EN, 1); // Enable Steering
			digitalWrite(steering_RPWM, 1);
			digitalWrite(steering_LPWM, 0);
			position = (analogRead(100)) / 3;		// Re-check position.
		}
		
		
	}
	else{
	// Stops actuator once it reaches proper position
	digitalWrite(steering_EN, 0); //disable steering
	digitalWrite(steering_RPWM, 0);
	digitalWrite(steering_LPWM, 0);
	cout << "\nDONE STEERING" <<endl;
}
	
}    

void run_braking(int intBrake, string &strBrake)
{
	digitalWrite(25, 1); //enable braking
	if (intBrake >= 24000) 
	{
		cout <<"\nBRAKING NOW";
		//digitalWrite(24, 0);		// EN1 and EN2 signal to initate brake.
		//digitalWrite(23, 1);
		//digitalWrite(28, 0); 	//disable Baccel
		//digitalWrite(29, 0); 	//disable Faccel
		softPwmWrite(7, 0);		// Shut off accel. motor
		strBrake = "ON";
	}

	else  
	{
		//digitalWrite(24, 1);
		//digitalWrite(23, 0);
		strBrake = "Off";
		cout <<"\nDONE BRAKING";
	}
	
}

void run_acceleration(int intAccel)
{
	intAccel = 32767 - intAccel;
	int localAccel = map(intAccel, 0, 32639, 0, 100);
	//cout << localAccel;
	//output controls
	if (localAccel == 0){
		digitalWrite(drive_REN, 0); //disable Baccel
		digitalWrite(drive_LEN, 0); //disable Faccel
		softPwmWrite(drive_PWM, 0);
		}
		
	else {
	digitalWrite(drive_REN, 1); //enable Baccel
	digitalWrite(drive_LEN, 1); //enable Faccel

	//cout << "\nRUNNING ACCEL"<<endl;
	
	//intAccel = ((intAccel) * 500 / 65535);
	
	softPwmWrite(drive_PWM, localAccel);
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


