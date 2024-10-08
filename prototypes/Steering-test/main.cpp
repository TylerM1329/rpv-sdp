// Basic C++ Libraries
#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <unistd.h>
//Pi's GPIO Configuration
#include <wiringPi.h>	// For control signals sent to motor drivers
#include <softPwm.h>	// For PWM signal generation (sub-library of wiring pi)
#include <mcp3004.h>	// The ADC Configuration header
using namespace std;

const char steering_RPWM = 24;
const char steering_LPWM = 23;
const char steering_EN = 13;

const char drive_RPWM = 17;
const char drive_LPWM = 18;
const char drive_REN = 21;
const char drive_LEN = 20;

int position = 0;
int targetPos = 0;
int error = 0;
int buffer = 0;

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
	
	//ADC Setup
	//mcp3004Setup(100, 0);
}
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
int main() {
    if (wiringPiSetupGpio() == -1)
		cout << "Error" << endl;
    else {
      initialize_PI();
      mcp3004Setup(100, 0);
    }
    //centering logic here
	while(1){
	cout << analogRead(100) << endl;
	}
    /*while(true) {
      	position = analogRead(100);     // get current position
		targetPos = map(analogRead(101), 0, 50, 104, 424);
		error = abs(position - targetPos);
		cout << "POSITION " << position << " ERROR " << error << " TARGET " << targetPos << endl;
		if (1) {
			if (position + buffer < targetPos) {
				position = analogRead(100);     // get current position
				//cout << "TURN LEFT" << endl;
				digitalWrite(steering_EN, 1);
		    	digitalWrite(steering_RPWM, 0);
		    	digitalWrite(steering_LPWM, 1);
			} else if (position - buffer > targetPos) {
				position = analogRead(100);     // get current position
				//cout << "TURN RIGHT" << endl;
				digitalWrite(steering_EN, 1);
		    	digitalWrite(steering_RPWM, 1);
		    	digitalWrite(steering_LPWM, 0);
			}
		} else {
			digitalWrite(steering_EN, 0);
		    digitalWrite(steering_RPWM, 0);
		    digitalWrite(steering_LPWM, 0);
		}
		//usleep(50000);
    }*/
    return 0;
}
