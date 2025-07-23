# DIY LiDAR-Based Autonomous Mapping and Navigation Robot

A remote-controlled Arduino car with a 2D LiDAR array and Python data visualization, built for a Robotics 206 microcontrollers course at Nazarbayev University.


## Overview

This project is an educational prototype for a robot vacuum cleaner platform. The robot is built using Arduino, equipped with a custom LiDAR system, and communicates wirelessly with a remote controller. The remote relays sensor data to a laptop, where Python software visualizes the robot's environment in real time. The system supports both manual joystick control and (planned) autonomous navigation.

**Note:** Autonomous navigation was not completed in time; the project currently supports manual mode and real-time mapping.



## System Architecture

- **Robot Car (Arduino Mega 2560)**
  - Drives motors and collects sensor data (gyroscope, accelerometer, compass, LiDAR/ultrasonic).
  - Communicates with the remote via NRF24L01+ radio module.
  - Sends sensor data and receives movement commands.

- **Remote Controller (Arduino Nano)**
  - Reads joystick and button inputs.
  - Communicates with both the robot (via NRF24L01+) and the laptop (via USB serial).
  - Relays data between the robot and the laptop.

- **Laptop (Python Backend)**
  - Receives sensor data from the remote over serial.
  - Visualizes robot position, heading, and obstacles in a 2D map using Pygame.
  - (Planned) Computes navigation paths and sends movement commands back to the robot.



## Features

- **Manual Mode:** Control the robot in real time using a joystick.
- **2D Mapping:** Visualizes LiDAR/ultrasonic sensor data and robot pose on a live map.
- **Bidirectional Communication:** All devices (robot, remote, laptop) communicate in both directions for control and feedback.
- **Modular Hardware:** Easy to debug and expand with breadboards and modular power supplies.



## Parts List

### Robot (communicates with remote)
- 1 x Arduino Mega 2560
- 1 x NRF24L01+ wireless module
- 1 x GY521 gyroscope & accelerometer
- 2 x GY-271 compass modules
- 1 x LM2596 DC-DC step-down power supply for Arduino
- 2 x large breadboards
- 1 x PD/QC/AFC Type-C fast charging trigger (up to 65W) with power bank

### Remote (communicates with laptop)
- 1 x Joystick board with buttons
- 1 x Arduino Nano
- 1 x NRF24L01+ wireless module

### Drivetrain
- 2 x VEX 2-wire motor 269
- 1 x L298n motor driver
- 3 x wheels
- VEX chassis components

### LiDAR/Sensors
- 1 x HS-225MG Mighty Mini Servo or EMAX ES09MA II Servo
- 1 x LM2596 DC-DC step-down power supply for servo
- 3 x GY-530 laser range-meters (replaced by ultrasonic sensors due to hardware failure)
- 3 x HC-SR04 ultrasonic sensors or VL53L0X



## Software Structure

- **Arduino Code (robot & remote):**
  - Handles sensor reading, motor control, radio communication, and serial data formatting.
- **Python Backend:**
  - `main.py`/`main2.py`: Handles serial communication, packet parsing, and data relay.
  - `my_model.py`: Visualizes robot pose and sensor data using Pygame, processes incoming LiDAR/ultrasonic readings.


## How It Works

1. **Robot** scans the environment with its sensors and sends data to the remote via RF.
2. **Remote** relays this data to the laptop over USB serial and receives control commands from the laptop or joystick.
3. **Laptop** visualizes the data in real time and (optionally) sends navigation commands back through the remote to the robot.


## Limitations

- Autonomous navigation and path planning were not implemented in time.
- LiDAR was initially planned with laser rangefinders, but due to hardware failure, ultrasonic sensors were used.



## Demo Video
Sorry for bad video quality. I had to compress it to 10 MB
<video width="630" height="300" src="https://github.com/user-attachments/assets/b62e1d9e-fedd-4c53-86b7-28f57e8e93c4"></video>



## File Descriptions

- `arduino/robot.ino` – Arduino code for the robot (sensors, motors, RF comms).
- `arduino/remote.ino` – Arduino code for the remote (joystick, buttons, RF & serial comms).
- `python/main.py` – Serial communication and packet parsing.
- `python/main2.py` – Alternative/extended serial communication logic.
- `python/my_model.py` – Pygame-based visualization of robot and sensor data.
- `README.md` – This file.



## How to Run

1. **Upload** the Arduino sketches to the robot and remote.
2. **Connect** the remote to your laptop via USB.
3. **Install** Python dependencies (see `requirements.txt` if available; mainly `pygame` and `pyserial`).
4. **Run** the Python visualization backend:
   ```
   python python/my_model.py
   ```
5. **Control** the robot using the joystick; watch the real-time map update as the robot moves.



## Educational Value

This project demonstrates:
- Wireless communication between microcontrollers and a PC.
- Real-time sensor data processing and visualization.
- Modular hardware prototyping and debugging.
- Integration of multiple sensor types for robotics applications.



## Credits

Developed as a group project for Robotics 206 at Nazarbayev University, 2nd year Computer Science.



