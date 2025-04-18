//
// Created by Kadir on 08.04.2025.
//

#ifndef STRUCTS_H
#define STRUCTS_H


// =============================
//      CONSTANTS & DEFINES
// =============================

// Pipe addresses for radio communication
#define CAR_RADIO_ADDRESS {'0', '0','0','0','0','1'}    // Car writes, remote reads
#define REMOTE_RADIO_ADDRESS {'0', '0','0','0','0','2'} // Remote writes, car reads

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};

// Start and end markers for structs sent over serial remote <-> pc
const byte SERIAL_START_MARKER = 0x7E;
const byte SERIAL_END_MARKER = 0x7F;

// Common loop delay for remote and car
// Needed to slow down loop() to avoid desyncs
constexpr int LOOP_WAIT = 50;

// Acceleration limiter for motors
constexpr int MAX_POWER_CHANGE_PER_CYCLE = 100;
const int MOTOR_MAX_INT = 250;
const float MOTOR_MAX_FLOAT = 250.0;


// =================
//      STRUCTS
// =================

// Joystick reading format
struct __attribute__((packed)) JoystickRead {
    short speed; // -255 = full reverse; 255 = full forward
    short turn;  // -255 = full left; 255 = full right
    bool k; // button
};


// gyro-521 (accel + gyro) reading format
struct __attribute__((packed)) GyroRead {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
};


// Lidar reading format
struct __attribute__((packed)) LidarRead {
    short angle;
    unsigned short distance;
};

// All three sensors at once
struct __attribute__((packed)) FullLidarRead {
    LidarRead sensor1;
    LidarRead sensor2;
    LidarRead sensor3;
    unsigned long timestamp;
};


// Car -> Remote
struct __attribute__((packed)) CarToRemote {
    // lidar data
    FullLidarRead lidar;

    // gy-273
    float azimuth;

    // gyro 521 data
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
    FullLidarRead lidar;

    // gy-273
    float azimuth;

    // gyro 521 data
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
