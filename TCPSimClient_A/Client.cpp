// Client application for sending steering, braking and acceleration data from racing simulator chair over IP
// Start up server on vehicle before entering in IP and port info into this application

// Last edit : CW 4/4/2022
// Added comments to include description of code.
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
#pragma comment(lib, "dinput8.lib")
#pragma comment(lib, "dxguid.lib")

using namespace std;
/////////////////////////////////////////////////////////////////////////////////////////
// DIRECTINPUT INITIALIZATION/////
// INITIALIZATION CODE WAS TAKEN FROM Jon Parise <jparise@cmu.edu/////
/////////////////////////////////////////////////////////////////////////////////////////
class Joystick
{
private:
	unsigned int            id;
	unsigned int            device_counter;
	LPDIRECTINPUT8          di;
	LPDIRECTINPUTDEVICE8    joystick;

public:
	Joystick(unsigned int id);
	~Joystick();

	HRESULT deviceName(char* name);

	HRESULT open();
	HRESULT close();

	HRESULT poll(DIJOYSTATE2* js);

	BOOL CALLBACK enumCallback(const DIDEVICEINSTANCE* instance, VOID* context);

	// Device Querying
	static unsigned int deviceCount();
};

BOOL CALLBACK enumCallback(const DIDEVICEINSTANCE* instance, VOID* context);
BOOL CALLBACK countCallback(const DIDEVICEINSTANCE* instance, VOID* counter);


#define SAFE_RELEASE(p)     { if(p) { (p)->Release(); (p) = NULL; } }

Joystick::Joystick(unsigned int id)
{
	this->id = id;
	device_counter = 0;

	di = NULL;
	joystick = NULL;
}

Joystick::~Joystick()
{
	close();
}

HRESULT
Joystick::deviceName(char* name)
{
	HRESULT hr;
	DIDEVICEINSTANCE device;

	ZeroMemory(&device, sizeof(device));
	device.dwSize = sizeof(device);

	if (!di || !joystick) {
		return E_INVALIDARG;
	}

	if (FAILED(hr = joystick->GetDeviceInfo(&device))) {
		return hr;
	}


	return hr;
}

HRESULT
Joystick::open()
{
	HRESULT hr;

	// Create a DirectInput device
	if (FAILED(hr = DirectInput8Create(GetModuleHandle(NULL),
		DIRECTINPUT_VERSION,
		IID_IDirectInput8,
		(VOID * *)& di, NULL))) {
		return hr;
	}

	// Look for the first simple joystick we can find.
	if (FAILED(hr = di->EnumDevices(DI8DEVCLASS_GAMECTRL, ::enumCallback,
		(LPVOID)this, DIEDFL_ATTACHEDONLY))) {
		return hr;
	}

	// Make sure we got a joystick
	if (joystick == NULL) {
		return E_FAIL;
	}

	// Set the data format to "simple joystick" - a predefined data format 
	//
	// A data format specifies which controls on a device we are interested in,
	// and how they should be reported. This tells DInput that we will be
	// passing a DIJOYSTATE2 structure to IDirectInputDevice::GetDeviceState().
	if (FAILED(hr = joystick->SetDataFormat(&c_dfDIJoystick2))) {
		return hr;
	}

	// Set the cooperative level to let DInput know how this device should
	// interact with the system and with other DInput applications.
	if (FAILED(hr = joystick->SetCooperativeLevel(NULL, DISCL_EXCLUSIVE | DISCL_FOREGROUND))) {
		return hr;
	}

	return S_OK;
}

HRESULT
Joystick::close()
{
	if (joystick) {
		joystick->Unacquire();
	}

	SAFE_RELEASE(joystick);
	SAFE_RELEASE(di);

	return S_OK;
}

HRESULT
Joystick::poll(DIJOYSTATE2* js)
{
	HRESULT hr;

	if (joystick == NULL) {
		return S_OK;
	}

	// Poll the device to read the current state
	hr = joystick->Poll();
	if (FAILED(hr)) {

		// DirectInput is telling us that the input stream has been
		// interrupted.  We aren't tracking any state between polls, so we
		// don't have any special reset that needs to be done.  We just
		// re-acquire and try again.
		hr = joystick->Acquire();
		while (hr == DIERR_INPUTLOST) {
			hr = joystick->Acquire();
		}

		// If we encounter a fatal error, return failure.
		if ((hr == DIERR_INVALIDPARAM) || (hr == DIERR_NOTINITIALIZED)) {
			return E_FAIL;
		}

		// If another application has control of this device, return success.
		// We'll just have to wait our turn to use the joystick.
		if (hr == DIERR_OTHERAPPHASPRIO) {
			return S_OK;
		}
	}

	// Get the input's device state
	if (FAILED(hr = joystick->GetDeviceState(sizeof(DIJOYSTATE2), js))) {
		return hr;
	}

	return S_OK;
}

BOOL CALLBACK
Joystick::enumCallback(const DIDEVICEINSTANCE* instance, VOID* context)
{
	// If this is the requested device ID ...
	if (device_counter == this->id) {

		// Obtain an interface to the enumerated joystick.  Stop the enumeration
		// if the requested device was created successfully.
		if (SUCCEEDED(di->CreateDevice(instance->guidInstance, &joystick, NULL))) {
			return DIENUM_STOP;
		}
	}

	// Otherwise, increment the device counter and continue with
	// the device enumeration.
	device_counter++;

	return DIENUM_CONTINUE;
}

BOOL CALLBACK
enumCallback(const DIDEVICEINSTANCE* instance, VOID* context)
{
	if (context != NULL) {
		return ((Joystick*)context)->enumCallback(instance, context);
	}
	else {
		return DIENUM_STOP;
	}
}

unsigned int
Joystick::deviceCount()
{
	unsigned int counter = 0;
	LPDIRECTINPUT8 di = NULL;
	HRESULT hr;

	if (SUCCEEDED(hr = DirectInput8Create(GetModuleHandle(NULL),
		DIRECTINPUT_VERSION,
		IID_IDirectInput8,
		(VOID * *)& di, NULL))) {
		di->EnumDevices(DI8DEVCLASS_GAMECTRL, ::countCallback,
			&counter, DIEDFL_ATTACHEDONLY);
	}

	return counter;
}

BOOL CALLBACK
countCallback(const DIDEVICEINSTANCE* instance, VOID* counter)
{
	if (counter != NULL) {
		unsigned int* tmpCounter = (unsigned int*)counter;
		(*tmpCounter)++;
		counter = tmpCounter;
	}

	return DIENUM_CONTINUE;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//END OF THE INITALIZATION /////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

int main()
{
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
	inet_pton(AF_INET, ipAddr, &server.sin_addr);
	//inet_pton(AF_INET, "::1", &server.sin_addr);

	//Socket Creation
	SOCKET stream_out = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);// SOCK_STREAM is TCP

	// Joystick Defined
	Joystick* joysticks[4]{};
	unsigned int i;
	unsigned int numJoysticks = Joystick::deviceCount();
	printf("Found %d joysticks:\n", numJoysticks);
	Sleep(1000);


	// Initialize all of the joysticks on the system.
	for (i = 0; i < numJoysticks; i++) {
		joysticks[i] = new Joystick(i);
		joysticks[i]->open();

		// Print the name of the joystick.
		char name[MAX_PATH];
		joysticks[i]->deviceName(name);
		printf("  Joystick %d: %s\n", i, name);
	}

	// Note: The following code only reads from the first joystick.
	while (true)
	{
		system("CLS");
		DIJOYSTATE2 js;
		DIJOYSTATE2 js2;

		Sleep(1);
		joysticks[1]->poll(&js);
		joysticks[0]->poll(&js2);
		//Converting Long to Char for sending data////
		char simData[18];
		int simDataLen = 18;
		//snprintf(char, charsize, string (if use variables for string)) this takes any string put it into a char which seems needed to parse
		// char seems easier to parse snprintf returns the number char written

		//int retSimDataSize = snprintf(simData, simDataLen, "S:%ld G:%ld B:%ld", js.lX, js.lRz, js.lY); - Allen
		//changed to gas and brake for xbox 360 controller
		//int retSimDataSize = snprintf(simData, simDataLen, "S:%ld A:%ld", js.lX, js.lZ);
		
		printf("js.lX : %d ...\n", js.lX); //Steering Wheel
		printf("js.lRz : %d ...\n", js.rglSlider[0]); // Accelerator
		printf("js.lY : %d ...\n", js.lY); //Brake
		printf("js.lRz : %d ...\n", js.lRz); //Clutch

		/*for (int i = 89; i < 128; i++) {
			printf("rgbButtons[%d] = %d\n", i, js.rgbButtons[i]);
		}*/

		printf("js2.lX : %x...\n", js2.rgbButtons[0]);
		printf("js2.lX : %x...\n", js2.rgbButtons[7]);

		int fwd = js2.rgbButtons[0];
		int rvs = js2.rgbButtons[7];

		int camR = js.rgbButtons[18];
		int camL = js.rgbButtons[17];
		int camC = js.rgbButtons[4];

		int RT = js.rgbButtons[1];
		int LT = js.rgbButtons[0]; 

		int scroll = js.rgbButtons[5];
		int flash = js.rgbButtons[9];

		int tempBtns1 = 0;
		if (camR)
			tempBtns1 = 2;
		else if (camL)
			tempBtns1 = 1;
		else if (camC)
			tempBtns1 = 3;

		int tempBtns2 = 0;
		if (RT)
			tempBtns2 = 1;	// right blinker
		else if (LT)
			tempBtns2 = 2;	// left blinker
		else if (scroll)
			tempBtns2 = 3;	// CV control
		else if (flash)
			tempBtns2 = 4;	// headlights



		printf("Foward : %d \n", fwd);
		printf("Reverse : %d \n", rvs);

		//STEERING
		int newTempSteer = 0;
		double tempSteer = (double)js.lX;
		//temp = round(100 - ((65600 - temp) / 65600.0 * 100)); //edit 11/2 - AC 
		//tempSteer = round((65600 - tempSteer) / 65600.0 * 100); // edit 11/7 11:28 - AC
		//int newTempSteer = (int)tempSteer;
		if (tempSteer >= 19717 && tempSteer <= 46000)
			newTempSteer = map(tempSteer, 46000, 19717, 0, 100);
		else if (tempSteer > 46000)
			newTempSteer = 0;
		else
			newTempSteer = 100;

		//////////////

		//// Display joystick state to dialog
		printf("Steering: %ld\n", newTempSteer); //32,767 is center therefore its straight
		
		//ACCELERATION

		double tempAccel = (double)js.lRz;

		tempAccel = round((65600 - tempAccel) / 65600.00 * 100);

		int newTempAccel = (int)tempAccel;

		printf("Acceleration: %d\n", newTempAccel);

		//BRAKING

		double tempBraking = (double)js.lY;
		tempBraking = round((65600 - tempBraking) / 65600.00 * 100);

		int newTempBraking = (int)tempBraking;

		printf("Braking: %d\n", newTempBraking);
		if (fwd == 128)
		{
			int retSimDataSize = snprintf(simData, simDataLen, "F-%3d-%3d-%3d-%1d-%1d", newTempSteer, newTempAccel, newTempBraking, tempBtns1, tempBtns2);
		}
		else if (rvs == 128)
		{
			int retSimDataSize = snprintf(simData, simDataLen, "R-%3d-%3d-%3d-%1d-%1d", newTempSteer, newTempAccel, newTempBraking, tempBtns1, tempBtns2);
		}
		else
		{
			int retSimDataSize = snprintf(simData, simDataLen, "F-%3d-%3d-%3d-%1d-%1d", newTempSteer, newTempAccel, newTempBraking, tempBtns1, tempBtns2);
		}
		
		char* sim_data = simData;


		//TCP connection
		connect(stream_out, (SOCKADDR*)& server, sizeof(server));
		printf("Sent: %s\n", simData);
		send(stream_out, simData, sizeof(simData), 0);
		printf("Size of message: %d", sizeof(simData));

		Sleep(50);

	}

	// Close the joysticks.
	for (i = 0; i < numJoysticks; i++) {
		joysticks[i]->close();
	}
	//Clean ipAddr Mem
	ZeroMemory(ipAddr, sizeof(ipAddr));
	//close the socket
	closesocket(stream_out);

	//Close down WinSock
	WSACleanup();
	return 0;
}