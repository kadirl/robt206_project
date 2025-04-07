//
// Created by Kadir on 08.04.2025.
//

#ifndef SHARED_H
#define SHARED_H

struct SensorData {
    // lidar data
    int16_t angle;
    int16_t distance;

    // gyro 521 data
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
};

#endif // SHARED_H

