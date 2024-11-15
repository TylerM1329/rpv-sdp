### Remote Pilotless Vehicle EGR4820/4830 (Senior Design Project) at California State Polytechnic University, Pomona

This branch is dedicated to the testing purposes of this project

Some of the source code was reused from previous years, and future teams may reuse code from this repository. It contains the source code for the various systems used in the operation of the vehicle.

### Directories and their usage:
* **RPV-Server** is a VS Code project containing the main server software that runs on the vehicle's Raspberry Pi.
* **XMPP-Client** is a directory containing the Python script for the XMPP client. This sends vehicle telemetry data to the Roadside Unit using messages through the XMPP protocol.
* **TCPSimClient_A** is a Visual Studio 2019 project that interfaces with the VRX simulator. It only understands inputs from the steering wheel, pedals, and gear shifter. It does not talk to any motion or haptics hardware.
* **TCPSimClient_B** is a Visual Studio 2019 project that interfaces with an Xbox controller and sends control information to the server. It can only run in a Windows environment as the libraries used are built for Windows.
* **TCPSimClient_C** is a Visual Studio 2019 project that listens for keyboard presses and sends control information to the server. It can only run in a Windows environment as the libraries used are built for Windows.
* **subsystems** is a directory containing the Arduino projects for each subsystem. The steering subsystem is not used, but is left for reference.
* **OpenCV** is a directory containing the Python script that runs lane detection using the OpenCV library. It was written using the PyCharm IDE. 

The remaining files are prototypes or scratchwork that have no immediate use.

## WARNING: GPS only works if you get 4 or more satellites for calculations