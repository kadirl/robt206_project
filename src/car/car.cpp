#include <RF24.h> // Include RF24 library for easier control
#include <Servo.h>
#include <structs.h>
#include <Utils.h>
#include <Arduino.h>
#include <QMC5883LCompass.h>
#include <MPU6050_light.h>
#include <NewPing.h>

// =================================
//      DEFINE PINS & CONSTANTS
// =================================

// RF-module pins
#define CE_PIN 9
#define CS_PIN 10

// Motor controller pins
#define MOTOR_LEFT_EN 5
#define MOTOR_LEFT_IN1 22
#define MOTOR_LEFT_IN2 24
#define MOTOR_RIGHT_EN 6
#define MOTOR_RIGHT_IN1 26
#define MOTOR_RIGHT_IN2 28

// Lidar servo pins
#define SERVO_PIN 4

// Lidar sensor pins
#define SONAR_OFFSET_1  -90
#define ECHO_PIN        36  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN     37  // Arduino pin tied to trigger pin on the ultrasonic sensor.

#define SONAR_OFFSET_2  30
#define ECHO_PIN_2      38  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN_2   39  // Arduino pin tied to trigger pin on the ultrasonic sensor.

#define SONAR_OFFSET_3  150
#define ECHO_PIN_3      40  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN_3   41  // Arduino pin tied to trigger pin on the ultrasonic sensor.

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


// ========================
//      INIT VARIABLES
// ========================

// Create RF24 object with CE pin 7, CSN pin 8
RF24 radio(CE_PIN, CS_PIN);

// Create a compass
QMC5883LCompass compass;
volatile bool compass_ready = false;

// MPU-6050
MPU6050 mpu(Wire);

// Create Servo object for lidar
Servo servo;
const int MIN_ANGLE = 25;  // The starting angle for the sweep (degrees)
const int MAX_ANGLE = 155; // The ending angle for the sweep (degrees)
// (MAX_ANGLE - MIN_ANGLE should be 120 degrees)
const int STEP_SIZE = 5;   // How many degrees to move in each step
const int STEP_DELAY = 30; // Delay in milliseconds between each step.
// Adjust this value if the servo moves too fast/jerky
// or too slow. Typical values: 15-50ms.
int currentAngle = MIN_ANGLE; // Holds the current target angle for the servo
int direction = 1;            // Direction of sweep: 1 for forward (MIN to MAX), -1 for backward (MAX to MIN)
unsigned long lastStepTime = 0; // Stores the time (in ms) when the last step was taken

// Create sonars
NewPing sonar1(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
unsigned long lastPing = 0;
unsigned long pingDelay = 60;

// Init structs for sending and receiving data
RemoteToCar receive;
CarToRemote send;
// FIFOBuffer<CarToRemote, 10> carDataBuffer;

// State variables
bool sending = false;  // false by deafult, as it must receive first
unsigned long timestamp = millis();

// send data
unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 25; // send once per second
bool newData = false;



// ===================
//      FUNCTIONS
// ===================

void driveMotors(int pwmPin, int inPin1, int inPin2, int power) {
    if (power >= 0) {
        digitalWrite(inPin1, HIGH);
        digitalWrite(inPin2, LOW);
        analogWrite(pwmPin, power);
    } else if (power < 0) {
        digitalWrite(inPin1, LOW);
        digitalWrite(inPin2, HIGH);
        analogWrite(pwmPin, -power);
    } else {
        digitalWrite(inPin1, LOW);
        digitalWrite(inPin2, LOW);
        analogWrite(pwmPin, 0);
    }
}


// Takes processed joystick inputs (-255 to +255)
// Outputs final motor commands via pointers
void mixDifferentialDrive(int speed, int turn, int& finalLeft, int& finalRight) {

    // 1. Use float for intermediate calculations
    float rawLeft = (float)speed + (float)turn;
    float rawRight = (float)speed - (float)turn;

    // 2. Calculate scaling factor
    float maxMagnitude = max(abs(rawLeft), abs(rawRight));
    float scale = 1.0;

    if (maxMagnitude > MOTOR_MAX_FLOAT) {
        scale = MOTOR_MAX_FLOAT / maxMagnitude;
    }

    // 3. Apply scaling
    rawLeft *= scale;
    rawRight *= scale;

    // 4. Convert back to integer (implicitly truncates, which is usually fine)
    finalLeft = (int)rawLeft;
    finalRight = (int)rawRight;

    // 5. Optional final clamp (safety net)
    finalLeft = constrain(finalLeft, -MOTOR_MAX_INT, MOTOR_MAX_INT);
    finalRight = constrain(finalRight, -MOTOR_MAX_INT, MOTOR_MAX_INT);
}

// --- How to use it in your Car code ---
void setMotors(JoystickRead data) {
    static int currentAppliedLeftPower = 0;
    static int currentAppliedRightPower = 0;

    int targetLeftPower;
    int targetRightPower;

    // Calculate the ideal target powers using the mixing function
    mixDifferentialDrive(data.speed, data.turn, targetLeftPower, targetRightPower);

    // --- Smoothing Logic (Apply AFTER mixing) ---
    // Calculate the change needed for each motor
    int leftDelta = targetLeftPower - currentAppliedLeftPower;
    int rightDelta = targetRightPower - currentAppliedRightPower;

    // Limit the change per cycle (acceleration/deceleration limit)
    leftDelta = constrain(leftDelta, -MAX_POWER_CHANGE_PER_CYCLE, MAX_POWER_CHANGE_PER_CYCLE);
    rightDelta = constrain(rightDelta, -MAX_POWER_CHANGE_PER_CYCLE, MAX_POWER_CHANGE_PER_CYCLE);

    // Apply the limited change to the current power
    currentAppliedLeftPower += leftDelta;
    currentAppliedRightPower += rightDelta;

    // Final safety clamp (might be redundant, but safe)
    currentAppliedLeftPower = constrain(currentAppliedLeftPower, -MOTOR_MAX_INT, MOTOR_MAX_INT);
    currentAppliedRightPower = constrain(currentAppliedRightPower, -MOTOR_MAX_INT, MOTOR_MAX_INT);
    // --- End Smoothing Logic ---

    // Debug prints (optional)
    // Serial.print(" Tgt L/R: "); Serial.print(targetLeftPower); Serial.print("/"); Serial.print(targetRightPower);
    // Serial.print(" | Cur L/R: "); Serial.print(currentAppliedLeftPower); Serial.print("/"); Serial.println(currentAppliedRightPower);

    // Drive the motors with the calculated (smoothed) power values
    driveMotors(MOTOR_LEFT_EN, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, currentAppliedLeftPower);
    driveMotors(MOTOR_RIGHT_EN, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, currentAppliedRightPower);
}


void updateMessage() {
    unsigned long currentTime = millis(); // Get the current time

    if (lastPing - currentTime > pingDelay) {
        send.lidar.sensor1.angle = currentAngle + SONAR_OFFSET_1;
        send.lidar.sensor1.distance = sonar1.ping_cm();

        send.lidar.sensor2.angle = currentAngle + SONAR_OFFSET_2;
        send.lidar.sensor2.distance = sonar2.ping_cm();

        send.lidar.sensor3.angle = currentAngle + SONAR_OFFSET_3;
        send.lidar.sensor3.distance = sonar3.ping_cm();

//        Serial.print("1Ping: ");
//        Serial.print(send.lidar.sensor1.distance); // Send ping, get distance in cm and print result (0 = outside set distance range)
//        Serial.print("\n2Ping: ");
//        Serial.print(send.lidar.sensor2.distance); // Send ping, get distance in cm and print result (0 = outside set distance range)
//        Serial.print("\n3Ping: ");
//        Serial.print(send.lidar.sensor3.distance); // Send ping, get distance in cm and print result (0 = outside set distance range)
//        Serial.println("cm\n");

        lastPing = currentTime;
        send.lidar.timestamp = lastPing;
    }

    // read compass values
    if (compass_ready) {
        compass_ready = false;  // Reset flag

        compass.read();
        int x = compass.getX();
        int y = -compass.getY();  // Flip Y axis

        float heading = atan2((float)y, (float)x) * (180.0 / PI);

        if (heading < 0) heading += 360.0;
        if (heading >= 360.0) heading -= 360.0;

        // Serial.print("Corrected Heading: ");
        // Serial.println(heading);

        send.azimuth = heading;
    }

    // read MPU-6050 values
    mpu.update();
    send.gyro.accelX = mpu.getAccX();
    send.gyro.accelY = mpu.getAccY();
    send.gyro.accelZ = mpu.getAccZ();
    send.gyro.gyroX = mpu.getGyroX();
    send.gyro.gyroY = mpu.getGyroY();
    send.gyro.gyroZ = mpu.getGyroZ();

    send.timestamp = millis();

    // carDataBuffer.enqueue(send);
}


void sendToCar() {
    bool rslt;
    rslt = radio.write( &send, sizeof(CarToRemote) );
    // Always use sizeof() as it gives the size as the number of bytes.
    // For example if dataToSend was an int sizeof() would correctly return 2

    Serial.print("Data Sent ");
    // Serial.print(dataToSend);
    if (rslt) {
        if ( radio.isAckPayloadAvailable() ) {
            radio.read(&receive, sizeof(RemoteToCar));

            newData = true;
        }
        else {
            Serial.println("  Acknowledge but no data ");
        }
        // updateMessage();
    }
    else {
        Serial.println("  Tx failed");
    }

    prevMillis = millis();
}


void processReceivedData() {
    if (newData == true) {
        // Serial.print("  Acknowledge data ");
        // Serial.print(receive.joystick.speed);
        // Serial.print(", ");
        // Serial.println(receive.joystick.turn);
        // Serial.println();

        setMotors(receive.joystick);

        newData = false;
    }
}

void dataReadyISR() {
    compass_ready = true;
}


// ===============
//      SETUP
// ===============

void setup() {
    Serial.begin(19200);

    // Setup radio
    byte readingPipe[6] = CAR_RADIO_ADDRESS;
    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.enableAckPayload();
    // 5 gives a 1500 µsec delay which is needed for a 32 byte ackPayload
    radio.setRetries(5,5); // delay, count
    radio.openWritingPipe(readingPipe);
    Serial.println("SimpleTxAckPayload Starting");

    // Setup GY-273 compass
    compass.init();
    compass.setMode(0x01, 0x08, 0x10, 0x80);
    compass.setCalibrationOffsets(-271.00, -71.00, 401.7);
    compass.setCalibrationScales(1.31, 0.65, 1.42);
    pinMode(19, INPUT);
    attachInterrupt(digitalPinToInterrupt(19), dataReadyISR, RISING);  // Trigger when DRDY goes LOW
    compass.read();

    // Setup MPU-6050
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

    // Setup servo
    servo.attach(4);
    servo.write(MIN_ANGLE);
    lastStepTime = millis();
    delay(1000);

    // Setup motors
    pinMode(MOTOR_RIGHT_EN, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);
    pinMode(MOTOR_LEFT_EN, OUTPUT);
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);
}


// ==============
//      LOOP
// ==============

void loop() {
    unsigned long currentTime = millis(); // Get the current time

    updateMessage();

    currentMillis = millis();
    if (currentMillis - prevMillis >= txIntervalMillis) {
        sendToCar();
    }

    processReceivedData();

    if (currentTime - lastStepTime >= STEP_DELAY) {
        // It's time to take the next step
        lastStepTime = currentTime; // Record the time of this step

        // Calculate the next angle based on the direction
        currentAngle += direction * STEP_SIZE;

        // --- Check Boundaries and Change Direction ---
        if (direction == 1 && currentAngle >= MAX_ANGLE) {
            // Reached or exceeded the maximum angle while moving forward
            currentAngle = MAX_ANGLE; // Clamp to the maximum angle
            servo.write(currentAngle); // Ensure servo is written to the final position
            // Serial.print("Reached MAX Angle: "); Serial.println(currentAngle); // Debug
            direction = -1; // Change direction to backward
            // Serial.println("Pausing at MAX..."); // Debug
        } else if (direction == -1 && currentAngle <= MIN_ANGLE) {
            // Reached or exceeded the minimum angle while moving backward
            currentAngle = MIN_ANGLE; // Clamp to the minimum angle
            servo.write(currentAngle); // Ensure servo is written to the final position
            // Serial.print("Reached MIN Angle: "); Serial.println(currentAngle); // Debug
            direction = 1; // Change direction to forward
            // Serial.println("Pausing at MIN..."); // Debug
        } else {
            // We are within the sweep range, just move the servo
            servo.write(currentAngle);
            // Serial.print(" Angle: "); Serial.println(currentAngle); // Debug
        }
    }
}
