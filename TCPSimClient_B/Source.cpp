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
		(VOID**)&di, NULL))) {
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
		(VOID**)&di, NULL))) {
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
	char ipAddr[256];
	char port_Numb[256];
	int port_num;
	//printf("Enter in IP address: ");
	//fgets(ipAddr, sizeof(ipAddr), stdin);
	//printf("Enter in Port Number: ");
	//fgets(port_Numb, sizeof(port_Numb), stdin);
	//printf("Now sending data to %s ", ipAddr);
	//printf("Port Number: %s ", port_Numb);
	//port_num = stoi(port_Numb);
	//Sleep(3000);

	//Create a hint structure for the server
	sockaddr_in server;
	server.sin_family = AF_INET;
	server.sin_port = htons(54000);
	//server.sin_port = htons(10000);
	//inet_pton(AF_INET, "", &server.sin_addr); for use without VPN
	inet_pton(AF_INET, "192.168.5.132", &server.sin_addr); // 192.168.5.132 10.8.0.2
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
		//DIJOYSTATE2 js2;

		Sleep(1);
		joysticks[0]->poll(&js);
		//joysticks[0]->poll(&js2);
		//Converting Long to Char for sending data////
		char simData[18];
		int simDataLen = 18;
		//snprintf(char, charsize, string (if use variables for string)) this takes any string put it into a char which seems needed to parse
		// char seems easier to parse snprintf returns the number char written

		//int retSimDataSize = snprintf(simData, simDataLen, "S:%ld G:%ld B:%ld", js.lX, js.lRz, js.lY); - Allen
		//changed to gas and brake for xbox 360 controller
		//int retSimDataSize = snprintf(simData, simDataLen, "S:%ld A:%ld", js.lX, js.lZ);

		printf("js.lX : %d ...\n", js.lX); //Steering Wheel
		printf("js.lZ : %d ...\n", js.lZ); // Accelerator
		printf("js.lY : %d ...\n", js.lY); //Brake
		printf("js.lRx : %d ...\n", js.lRx);
		printf("js.lRy : %d ...\n", js.lRy);
		printf("js.lRz : %d ...\n", js.lRz);


		//printf("js2.lX : %x...\n", js2.rgbButtons[0]);
		//printf("js2.lX : %x...\n", js2.rgbButtons[7]);

		//int fwd = js2.rgbButtons[0];
		//int rvs = js2.rgbButtons[7];

		int btnX = js.rgbButtons[2];
		int btnB = js.rgbButtons[1];
		int btnA = js.rgbButtons[0];
		int btnRB = js.rgbButtons[5];
		int btnLB = js.rgbButtons[4];
		int btnSTART = js.rgbButtons[7];
		int btnBACK = js.rgbButtons[6];

		//printf("RB: %d, LB: %d", btnRB, btnLB);

		int tempBtns2 = 0;
		if (btnRB)
			tempBtns2 = 1;
		else if (btnLB)
			tempBtns2 = 2;
		else if (btnSTART)
			tempBtns2 = 3;
		else if (btnBACK)
			tempBtns2 = 4;

		printf("tempBtns2 : %d ...\n", tempBtns2);

		int tempBtns1 = 0;					// default value (nothing pressed)
		if (btnX)
			tempBtns1 = 1;
		else if (btnB)
			tempBtns1 = 2;
		else if (btnA)
			tempBtns1 = 3;

		printf("tempBtns1 : %d ...\n", tempBtns1);

		int fwd = 128;
		int rvs = 0;

		//STEERING///////////////////////////////////////////////

		double tempSteer = (double)js.lX;
		tempSteer = round((65600 - tempSteer) / 65600.0 * 100);
		int newTempSteer = (int)tempSteer;
		printf("Steering: %ld\n", newTempSteer);

		//ACCELERATION///////////////////////////////////////////

		double tempAccel = (double)js.lZ;
		if (tempAccel > 32770) {		// 32771 ~ 65408
			printf("Reverse\n");
			rvs = 128;
			fwd = 0;
			tempAccel = map(tempAccel, 32771, 65408, 0, 100);
		}
		else if (tempAccel < 32760) {		// 32761 ~ 128 -> 32761 - tempAccel -> 0 ~ 32633
			printf("Forward\n");
			fwd = 128;
			rvs = 0;
			tempAccel = map(32761 - tempAccel, 0, 32633, 0, 100);
		}
		else {
			// throttle zero, forward
			printf("No Throttle\n");
			tempAccel = 0;
		}

		//tempAccel = round((65600 - tempAccel) / 65600.00 * 100);
		int newTempAccel = (int)tempAccel;
		printf("Acceleration: %d\n", newTempAccel);
		printf("Foward : %d \n", fwd);
		printf("Reverse : %d \n", rvs);

		//BRAKING////////////////////////////////////////////////
		int newTempBraking = 0;
		double tempBraking = (double)js.lY;
		//tempBraking = round((65600 - tempBraking) / 65600.00 * 100);
		if (tempBraking < 29000) {
			// 29000~0
			newTempBraking = map(tempBraking, 29000, 0, 0, 100);
			printf("tempBraking = %d\n", newTempBraking);
		}
		else
			newTempBraking = 0;

		//int newTempBraking = (int)tempBraking;
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
		connect(stream_out, (SOCKADDR*)&server, sizeof(server));

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