#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

unsigned long counter = 0;
unsigned long diff = 0;
unsigned long start = 0;
unsigned long period = 1000;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(400000);

    byte status = mpu.begin();
    Serial.print("MPU6050 status: ");
    Serial.println(status);
    while (status != 0) { } // stop everything if there's an error

    Serial.println("Calibrating gyro...");
    delay(1000);
    mpu.calcGyroOffsets();  // Automatically calibrates at startup
    Serial.println("Done!\n");
}

void loop() {
    if (start == 0) {
        start = millis();
    } else if (millis() - start < period) {
        counter++;
    } else {
        Serial.print("READING PER SECONDS: ");
        Serial.println(counter);

        counter = 0;
        start = millis();
    }

    mpu.update();

    // Serial.print("Accel X: "); Serial.print(mpu.getAccX());
    // Serial.print("\tY: "); Serial.print(mpu.getAccY());
    // Serial.print("\tZ: "); Serial.println(mpu.getAccZ());

    // Serial.print("Gyro X: "); Serial.print(mpu.getGyroX());
    // Serial.print("\tY: "); Serial.print(mpu.getGyroY());
    // Serial.print("\tZ: "); Serial.println(mpu.getGyroZ());



    // Serial.println(mpu.getGyroZ());

    // delay(50);
}
