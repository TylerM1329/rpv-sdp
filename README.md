# Remote Pilotless Vehicle (RPV) EGR4820/4830  
**Senior Design Project**  
California State Polytechnic University, Pomona  

This repository contains the source code and related files for the Remote Pilotless Vehicle (RPV), developed as part of a senior design project at Cal Poly Pomona. The system includes various subsystems for controlling, monitoring, and communicating with the vehicle's hardware components, which include steering, telemetry, and ultrasonic sensors.

## Project Overview  
The goal of this project is to build a pilotless vehicle capable of autonomous movement, communication with external systems, and remote monitoring. The vehicle is controlled and monitored by various subsystems that work together to achieve full functionality, including steering, telematics, and obstacle avoidance.

This repository contains the necessary code for the vehicle's various subsystems, as well as tools for interfacing with a simulator or remote control.

### System Setup:
Follow the [RPV Setup Guide](https://github.com/TylerM1329/rpv-sdp/blob/main/RPV_setup_guide.pdf)

### Building the Project:  
While editing or updating the code, use `Ctrl + Shift + B` to build the project in `main.cpp` to compile and check for errors.

## Directory Structure  

### 1. `RPV-Server`  
This directory contains the main server software that runs on the Raspberry Pi of the vehicle. The server is responsible for receiving control commands and telemetry data from various subsystems, processing these commands, and sending the appropriate signals to the hardware. The server also handles communication with external systems.

- **Technologies**: C++, Visual Studio 2019
- **Key files**:
  - `main.cpp`: The main server script responsible for processing input from different systems.
  - `functions.cpp`: Contains helper functions used throughout the server code. These functions are used to handle specific tasks such as communication or sensor data processing.
  - `functions.h`: Header file declaring the functions used in `functions.cpp`. This ensures that these helper functions can be used across different parts of the server code.

### 2. `Subsystems`  
This directory contains the Arduino code for the various hardware subsystems of the vehicle, such as steering, telemetry, and ultrasonic sensors. Each subsystem is responsible for a specific function in the vehicle's operation.  

- **Key Subsystems**:
  - **Steering Subsystem**: Controls the steering mechanism based on inputs from the main server and maintains the steering position using a potentiometer.
  - **Lidar Subsystem**: Measures distances to obstacles using LiDAR sensor and sends this information via I²C to the main server.
  - **Telematics Subsystem**: Interfaces with a GPS module to provide real-time location data and heading information, sending this data via I²C to the main server.

Each subsystem is housed in its respective folder, where the relevant Arduino code can be found. Below are the key components of each subsystem:

#### Steering Subsystem (`steering_subsystem.ino`)
- **Function**: Controls the steering mechanism of the vehicle based on inputs from the main server.
- **Main Components**:
  - Reads from a steering potentiometer to determine the current steering position.
  - Controls motors using PWM signals to move the steering mechanism.
  - Listens for I²C commands to adjust the target steering position.
- **Key Libraries**: `Wire.h` (I²C communication)

#### Lidar Subsystem (`lidar_subsystem.ino`)
- **Function**: Measures distances to obstacles using a Lidar sensor and sends this data via I²C.
- **Main Components**:
  - Measures distances to objects using a Lidar sensor, providing more accurate and longer-range measurements compared to ultrasonic sensors.
  - Sends measured distances to the main server via I²C for processing.
- **Key Libraries**: `Wire.h` (I²C communication)
  
#### Telematics Subsystem (`telementary_subsystem.ino`)
- **Function**: Provides real-time GPS data, including latitude, longitude, and heading, and sends this data via I²C.
- **Main Components**:
  - Interfaces with a GPS module using the NMEA protocol to get location and heading data.
  - Sends GPS data over I²C when requested by the main server.
- **Key Libraries**: `NMEAGPS.h`, `SoftwareSerial.h`, `Wire.h` (I²C communication)

### 3. Controllers (`TCPSimClient_C`, `TCPSimClient_A`, `TCPSimClient_B`)
These controllers are used to send control commands to the vehicle's systems, such as steering, speed, and other vehicle functionalities. There are three controller types, each serving different purposes and hardware s.
  
  - **TCPSimClient_A**: 
    - **Purpose**: Interfaces with the VRX simulator and allows control via steering wheel, pedals, and gear shifter.
    - **Limitations**: Does not communicate with any motion or haptic hardware.
    - **Key Libraries**: Visual Studio 2019, Simulator-Specific Libraries.
  
  - **TCPSimClient_B**:
    - **Purpose**: Interfaces with an Xbox controller to send control information to the server.
    - **Limitations**: Only runs on Windows due to libraries built specifically for this environment.
    - **Key Libraries**: Visual Studio 2019, Xbox Controller Libraries.

  - **TCPSimClient_C** (Most Up-to-Date):
    - **Purpose**: This controller is the most advanced and is designed to interface with the vehicle's systems via keyboard inputs. It includes buttons for cruise control and autopilot functionalities.
    - **Features**: 
      - Includes buttons for enabling/disabling cruise control (C) and autopilot (V) modes.
      - Provides a more interactive and real-time control mechanism for the vehicle, making it the most up-to-date and feature-complete controller in the system.
    - **Limitations**: Only runs on Windows due to the use of libraries that are built specifically for Windows.
    - **Key Libraries**: Visual Studio 2019, Keyboard Input Libraries.

### Communication Protocols
- **I²C**: Used for communication between the Arduino subsystems and the main Raspberry Pi server.

## Future Enhancements
- **Sensor Fusion**: Integrating multiple sensors (e.g., ultrasonic, LiDAR) to improve obstacle detection and navigation.
- **Autonomous Navigation**: Developing algorithms for autonomous navigation using GPS, sensors, and path planning.
- **User Interface**: Developing a web-based or Python user interface for easier control and monitoring of the vehicle.

## Acknowledgments
- This project was developed as part of the Senior Design Project course at California State Polytechnic University, Pomona.
- Special thanks to all the faculty, advisors, and collaborators who contributed to the success of this project.

