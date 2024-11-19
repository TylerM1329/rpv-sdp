#include <WS2tcpip.h>
#include <string>
#include <windows.h>
#include <basetsd.h>
#include <dinput.h>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <iostream>
#include <iomanip>

#pragma comment (lib, "ws2_32.lib")

/*
Arrow left = left
Arrow right = right
Arrow up = forward
Arrow down = reverse
Space = brake
Q = camera left
W = camera center
E = camera right

Z = signal left
X = signal right
H = headlights
T = cv control
C = toggle cruise control
V = toggle autopilot

F-000-000-000-0-0
F-%3d-%3d-%3d-%1d-%1d

snprintf(simData, simDataLen, "F-%3d-%3d-%3d-%1d-%1d", newTempSteer, newTempAccel, newTempBraking, tempBtns1, tempBtns2);
*/
using namespace std;

int accelVal = 1;
int steering = 0;
int gear = 0;	// 0 = fwd, 1 = reverse
int accel = 0;
int brake = 0;
int tempBtns1 = 0;
int tempBtns2 = 0;

char txBuf[18];
int txBufLen = 18;

void get_network_options(char* ipAddr, int& port) {
	int source = 0;
connectionInput:
	cout << "What address are you connecting to? 1 for local, 2 for VPN, 3 for custom: ";
	cin >> source;
	switch (source)
	{
	case 1: {
		snprintf(ipAddr, 18, "192.168.5.132");
		port = 54000;
		break;
	}
	case 2: {
		snprintf(ipAddr, 18, "10.8.0.2");
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

int main() {
	// Startup WinSock
	WSADATA data;
	WORD version = MAKEWORD(2, 2);
	int wsOk = WSAStartup(version, &data);
	if (wsOk != 0)
	{
		printf("Can't Start WinSock %d", wsOk);
		return 0;
	}

	// Type in Ip Address and Port
	char ipAddr[15];
	int port = 0;
	get_network_options(ipAddr, port);
	printf("Accepted values: IP = %s Port = %d\n", ipAddr, port);
	system("PAUSE");

	//Create a hint structure for the server
	sockaddr_in server;
	server.sin_family = AF_INET;
	server.sin_port = htons(port);
	//server.sin_port = htons(10000);
	//inet_pton(AF_INET, "", &server.sin_addr); for use without VPN
	inet_pton(AF_INET, ipAddr, &server.sin_addr); // 192.168.5.132 10.8.0.2
	//inet_pton(AF_INET, "::1", &server.sin_addr);

	//Socket Creation
	SOCKET stream_out = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);// SOCK_STREAM is TCP

	while (1) {
		system("CLS");
		// STEERING AND THROTTLE KEYS
		if (GetKeyState(VK_LEFT) & 0x8000)
			steering = 100;		// left
		else if (GetKeyState(VK_RIGHT) & 0x8000)
			steering = 0;		// right
		else
			steering = 50;		// center
		if (GetKeyState(VK_DOWN) & 0x8000) {
			gear = 1;
			accel = accelVal;
		} else if (GetKeyState(VK_UP) & 0x8000) {
			gear = 0;
			accel = accelVal;
		} else {
			gear = 0;
			accel = 0;
		}
		// BRAKE KEYS
		if (GetKeyState(VK_SPACE) & 0x8000)
			brake = 100;
		else
			brake = 0;
		// CAMERA CONTROL KEYS
		if (GetKeyState('Q') & 0x8000) // pan camera left
			tempBtns1 = 1;
		else if (GetKeyState('E') & 0x8000) // pan camera right
			tempBtns1 = 2;
		else if (GetKeyState('W') & 0x8000) // camera center
			tempBtns1 = 3;
		else
			tempBtns1 = 0;
		
		
		// OTHER CONTROL KEYS
		if (GetKeyState('X') & 0x8000) // signal right
			tempBtns2 = 1;
		else if (GetKeyState('Z') & 0x8000) // signal left
			tempBtns2 = 2;
		else if (GetKeyState('T') & 0x8000) // cv control
			tempBtns2 = 3;
		else if (GetKeyState('H') & 0x8000) // headlights
			tempBtns2 = 4;
		else if (GetKeyState('C') & 0x8000) // cruise control
			tempBtns2 = 5;
		else if (GetKeyState('V') & 0x8000) // autopilot
			tempBtns2 = 6;
		else
			tempBtns2 = 0;

		printf("steering: %d ", steering);
		printf("gear: %d ", gear);
		printf("accel: %d ", accel);
		printf("brake: %d ", brake);
		printf("tempBtns1 (camera controls): %d ", tempBtns1);
		printf("tempBtns2 (other controls): %d\n", tempBtns2);
		
		if (gear)
			snprintf(txBuf, txBufLen, "R-%3d-%3d-%3d-%1d-%1d", steering, accel, brake, tempBtns1, tempBtns2);
		else
			snprintf(txBuf, txBufLen, "F-%3d-%3d-%3d-%1d-%1d", steering, accel, brake, tempBtns1, tempBtns2);
		printf("String sent: %s", txBuf);

		connect(stream_out, (SOCKADDR*)&server, sizeof(server));

		send(stream_out, txBuf, sizeof(txBuf), 0);

		Sleep(50);
	}
}