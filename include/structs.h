//
// Created by Kadir on 08.04.2025.
//

#ifndef STRUCTS_H
#define STRUCTS_H

#define CAR_RADIO_ADDRESS {'0', '0','0','0','0','1'}    // Car writes, remote reads
#define REMOTE_RADIO_ADDRESS {'0', '0','0','0','0','2'} // Remote writes, car reads

const byte SERIAL_START_MARKER = 0x7E;
const byte SERIAL_END_MARKER = 0x7F;
const int CYCLE_WAIT = 30;

const int MAX_POWER_CHANGE_PER_CYCLE = 50;


// Joystick reading format
struct __attribute__((packed)) JoystickRead {
    short speed; // -255 = full reverse; 255 = full forward
    short turn;  // -255 = full left; 255 = full right
    bool k; // button
};


// gyro-521 (accel + gyro) reading format
struct __attribute__((packed)) GyroRead {
    short accelX;
    short accelY;
    short accelZ;
    short gyroX;
    short gyroY;
    short gyroZ;
};


// Lidar reading format
struct __attribute__((packed)) LidarRead {
    unsigned short angle;
    unsigned short distance;
};


// Car -> Remote
struct __attribute__((packed)) CarToRemote {
    // lidar data
    bool lidar_updated;
    LidarRead lidar;

    // gyro 521 data
    bool gyro_updated;
    GyroRead gyro;

    // timestamp
    unsigned long timestamp;
};


// Remote -> Car
struct __attribute__((packed)) RemoteToCar {
    JoystickRead joystick;
    unsigned long timestamp;
};


// Remote -> PC
struct __attribute__((packed)) RemoteToPC {
    // lidar data
    bool lidar_updated;
    LidarRead lidar;

    // gyro 521 data
    bool gyro_updated;
    GyroRead gyro;

    // sensor reading timestamp
    unsigned long timestamp;

    // remote data
    bool mode;  // false - auto, true - manual
    JoystickRead joystick;
};


// PC -> Remote
struct __attribute__((packed)) PCToRemote {
    JoystickRead joystick;
};


#endif // STRUCTS_H
