#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>

const unsigned char RPWM = 5;
const unsigned char LPWM = 6;

void setupGPIO();

int main() {
	std::cout << "===== GPIO Test =====" << std::endl;
	setupGPIO();
	
	softPwmWrite(RPWM, 0);
	
	while (true) {
		softPwmWrite(LPWM, 25);
		std::cout << "25% speed" << std::endl;
		delay(2000);
		softPwmWrite(LPWM, 50);
		std::cout << "50% speed" << std::endl;
		delay(2000);
		softPwmWrite(LPWM, 75);
		std::cout << "75% speed" << std::endl;
		delay(2000);
		softPwmWrite(LPWM, 100);
		std::cout << "100% speed" << std::endl;
		delay(2000);
		
		for (int i = 0; i < 101; i++) {
			softPwmWrite(LPWM, i);
			std::cout << "PWM: " << i << std::endl;
			delay(150);
		}
		delay(1000);
		
	}
	return 0;
}

void setupGPIO() {
	wiringPiSetupGpio(); // Initializes wiringPi using 
						 // the Broadcom GPIO pin numbers
						 
	softPwmCreate(RPWM, 0, 100);
	softPwmCreate(LPWM, 0, 100);
	
}
