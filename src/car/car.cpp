#include <RF24.h> // Include RF24 library for easier control
#include <Servo.h>
#include <structs.h>
#include <Utils.h>
#include <Arduino.h>

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


// ========================
//      INIT VARIABLES
// ========================

// Create RF24 object with CE pin 7, CSN pin 8
RF24 radio(CE_PIN, CS_PIN);

// Create Servo object for lidar
Servo servo;
int angle = 0;
int step = 1;

// Init structs for sending and receiving data
RemoteToCar receive;
CarToRemote send;

// State variables
bool sending = false;  // false by deafult, as it must receive first
unsigned long timestamp = millis();
int failedToSend = 0;
int failedToSendMax = 5;


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


// ===============
//      SETUP
// ===============

void setup() {
    Serial.begin(19200);

    for (int i = 0; i < 20; i++) {
        Serial.print("RESETTING");
    }

    // Setup radio
    byte readingPipe[6] = REMOTE_RADIO_ADDRESS;
    byte writingPipe[6] = CAR_RADIO_ADDRESS;
    bool radioOk = setupRadio(radio, readingPipe, writingPipe);
    if (radioOk) {
        Serial.println("Car radio initialized successfully.");
    } else {
        Serial.println("Car radio initialization failed.");
    }

    // Setup servo
    servo.attach(4);

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
    // Send
    bool testr = radio.begin();
    if (testr) {
      Serial.println("Radio working.");
    } else {
      Serial.println("Radio dead");
    }

    Serial.print("Car sending: ");
    Serial.println(sending);

    if (sending) {
        radio.stopListening();

        // Prepare payload
        send.lidar_updated = true;
        send.gyro_updated = false;
        send.timestamp = millis();

        // Send payload
        bool success = radio.write(&send, sizeof(send));
        if (success) {
            Serial.print("Car sent payload at: ");
            Serial.println(send.timestamp);

            // Switch roles
            sending = !sending;
        } else {
            radio.flush_tx();
            Serial.println("Car failed to send payload.");
            // failedToSend++;
            //
            // if (failedToSend >= failedToSendMax) {
            //     sending = !sending;
            //     failedToSend = 0;
            //
            //     receive.joystick.speed = 0;
            //     receive.joystick.turn = 0;
            // }
        }

    // Receive
    } else {
        radio.startListening();
        // If there is anything to read
        if (radio.available()) {
            Serial.println("AAAAAAAAAAAAAaAVAILABLE");
            // Read
            radio.read(&receive, sizeof(receive));
            Serial.print("Car received payload at: ");

            Serial.println(millis() - timestamp);
            Serial.println(receive.joystick.speed);
            Serial.println(receive.joystick.turn);

            // Set motors
            // setMotors(receive.joystick);

            // Switch roles
            sending = !sending;
        } else {
            Serial.println("NO RADIO AVAILABLE");
        }
    }

    // if (angle == 0) {
    //     servo.write(0);
    //     angle = 1;
    // } else {
    //     servo.write(120);
    //     angle = 0;
    // }
    //
    // JoystickRead test = {255, 0, 0};
    // setMotors(test);
    //
    // delay(500);

    setMotors(receive.joystick);

    delay(LOOP_WAIT);
}
