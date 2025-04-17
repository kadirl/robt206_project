//
// Created by Kadir on 17.04.2025.
//


#include <QMC5883LCompass.h>

QMC5883LCompass compass;

volatile bool dataReady = false;
void dataReadyISR() {
    dataReady = true;
}


void setup() {
    Serial.begin(9600);

    Serial.println("setup");

    compass.init();

    compass.setMode(0x01, 0x08, 0x10, 0x80);

    compass.setCalibrationOffsets(-271.00, -71.00, 401.7);
    compass.setCalibrationScales(1.31, 0.65, 1.42);

    pinMode(19, INPUT);
    attachInterrupt(digitalPinToInterrupt(19), dataReadyISR, RISING);  // Trigger when DRDY goes LOW


    compass.read();
}

void loop() {

    // // Read compass values
    // compass.read();

    // compass.read();
    // int x = compass.getX();
    // int y = -compass.getY();  // Flip Y axis

    // float heading = atan2((float)y, (float)x) * (180.0 / PI);

    // // Normalize to 0–360 degrees
    // if (heading < 0) heading += 360.0;
    // if (heading >= 360.0) heading -= 360.0;

    // Serial.print("Corrected Heading: ");
    // Serial.println(heading);

    // delay(200);
    // Serial.print("DRDY State: ");
    // Serial.println(digitalRead(19));  // Should go LOW when new data is ready
    // compass.read();


    if (dataReady) {
        dataReady = false;  // Reset flag

        compass.read();
        int x = compass.getX();
        int y = -compass.getY();  // Flip Y axis

        float heading = atan2((float)y, (float)x) * (180.0 / PI);

        if (heading < 0) heading += 360.0;
        if (heading >= 360.0) heading -= 360.0;

        Serial.print("Corrected Heading: ");
        Serial.println(heading);
    }
}