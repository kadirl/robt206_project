#include <RF24.h> // Include RF24 library for easier control
#include <structs.h>
#include <Utils.h>
#include <Arduino.h>
#include <Bounce2.h>

// =================================
//      DEFINE PINS & CONSTANTS
// =================================

// Joystick pins
#define JOYSTICK_X_PIN 0
#define JOYSTICK_Y_PIN 1
#define JOYSTICK_K_PIN 8

// RF-module pins
#define CE_PIN 9
#define CS_PIN 10

// Serial RX buffer
#define SERIAL_RX_BUFFER_SIZE 64


// ========================
//      INIT VARIABLES
// ========================

// Create RF24 object with CE and CSN pins
RF24 radio(CE_PIN, CS_PIN);

// Create a button object for joystick button debouncing
Bounce2::Button button = Bounce2::Button();

// Init structs for sending and receiving data
CarToRemote     receiveFromCar;
RemoteToCar     sendToCar;
RemoteToPC      sendToPC;
PCToRemote      receiveFromPC;
JoystickRead    joystick;

// State variables
bool sending = true;    // true by default, as it is the first to send data
bool mode = true;       // true = manual; false = auto
bool newPCDataAvailable = false;    // Flag for new command from PC

// Variables for  serial read from PC
byte serialRxBuffer[SERIAL_RX_BUFFER_SIZE];
uint8_t serialRxIndex = 0;
bool receivingSerial = false;
int failedToSend = 0;
int failedToSendMax = 5;


// ===================
//      FUNCTIONS
// ===================

JoystickRead readJoystick(int xPin, int yPin) {
    int x = analogRead(xPin); // speed
    int y = analogRead(yPin); // turn

    button.update();
    int k = button.pressed();

    // convert from 0...650 to -255...255
    int speed = map(x, 0, 650, -255, 255);
    int turn = map(y, 0, 650, -255, 255);

    // correct for joystick drift
    // dead zone values obtained empirically
//    JoystickRead data = {
//        // correct for joystick drift
//        // dead zone values obtained empirically
//        static_cast<short>(constrain(-(speed < 20 && speed > -20 ? 0 : speed), -255, 255)),
//        static_cast<short>(constrain((turn < 20 && turn > -20 ? 0 : turn), -255, 255)),
//        static_cast<bool>(k)
//    };

    JoystickRead data = {
        // correct for joystick drift
        // dead zone values obtained empirically
        static_cast<short>(constrain(-speed, -255, 255)),
        static_cast<short>(constrain(turn, -255, 255)),
        static_cast<bool>(k)
    };

    return data;
}

void sendSerialToPC(RemoteToPC data) {
    Serial.write(SERIAL_START_MARKER);
    Serial.write(reinterpret_cast<byte *>(&data), sizeof(data));
    Serial.write(SERIAL_END_MARKER);
}


// Function to read serial data from PC non-blockingly
// Returns true if a complete, valid packet was received THIS CALL
bool readSerialFromPC() {
    bool packetReceived = false;
    static const uint8_t packetSize = sizeof(PCToRemote); // Size of expected data payload

    while (Serial.available() > 0) {
        byte incomingByte = Serial.read();

        if (!receivingSerial) {
            // Waiting for the start marker
            if (incomingByte == SERIAL_START_MARKER) {
                receivingSerial = true;
                serialRxIndex = 0; // Reset buffer index
                // Serial.println("DEBUG: Start marker found"); // Debug
            }
            // Else: Ignore bytes before start marker
        } else {
            // Already found start marker, collecting payload or looking for end marker
            if (serialRxIndex < packetSize) {
                // Still collecting payload bytes
                serialRxBuffer[serialRxIndex++] = incomingByte;
            } else {
                // Have collected payload bytes, expect end marker now
                if (incomingByte == SERIAL_END_MARKER) {
                     // Serial.println("DEBUG: End marker found"); // Debug
                     // Valid packet received
                     // Copy buffer contents into the target struct
                     memcpy(&receiveFromPC, serialRxBuffer, packetSize);
                     packetReceived = true; // Signal that new data is ready
                     // Serial.print("DEBUG: Received from PC: Spd="); // Debug
                     // Serial.print(receiveFromPC.target_speed); // Debug
                     // Serial.print(", Turn="); // Debug
                     // Serial.println(receiveFromPC.target_turn); // Debug

                } else {
                     // Error: Expected end marker, got something else
                     Serial.print("Serial RX Error: Expected END_MARKER, got 0x");
                     Serial.println(incomingByte, HEX);
                }
                // Reset state machine whether end marker was correct or not
                receivingSerial = false;
                serialRxIndex = 0;
            }
        }
    }

    // Handle potential buffer overflow if end marker never arrives
    if (receivingSerial && serialRxIndex >= SERIAL_RX_BUFFER_SIZE) {
         Serial.println("Serial RX Error: Buffer overflow!");
         receivingSerial = false;
         serialRxIndex = 0;
    }

    return packetReceived;
}


// ===============
//      SETUP
// ===============

void setup() {
    Serial.begin(9600);

    // Setup joystick button
    button.attach (JOYSTICK_K_PIN , INPUT_PULLUP);
    button.interval(5);
    button.setPressedState(LOW);

    // Setup radio
    byte readingPipe[6] = CAR_RADIO_ADDRESS;
    byte writingPipe[6] = REMOTE_RADIO_ADDRESS;
    bool radioOk = setupRadio(radio, readingPipe, writingPipe);

    // Hidden to avoid interference with Serial communication
    if (radioOk) {
        Serial.println("Remote radio initialized successfully.");
    } else {
        Serial.println("Remote radio initialization failed.");
    }
}


// ==============
//      LOOP
// ==============

void loop() {
    bool testr = radio.begin();
    if (testr) {
        Serial.println("Radio working.");
    } else {
        Serial.println("Radio dead");
    }
    // START — READ SENSORS & RECEIVE DATA

    // Receive data from PC over Serial
    newPCDataAvailable = readSerialFromPC();

    // Read joystick data
    joystick = readJoystick(JOYSTICK_X_PIN, JOYSTICK_Y_PIN);
    if (joystick.k) {
        mode = !mode;
    }

    // END — READ SENSORS & RECEIVE DATA


    // START — COMMUNICATE WITH CAR AND PC

    if (sending) {
        // Receive data from PC over serial
        // and redirect it to the car over radio

        // Stop listening for signals from the car
        radio.stopListening();

        // === PREPARE PAYLOAD ===
        // Manual/auto  switch selector
        // true = manual mode, joystick input; false = auto mode, PC input
        // mode variable updated in the beginning of loop()

        // Manual mode
        if (mode) {
            // Send the onboard joystick data to the car
            // joystick variable is updated in the beginning of the loop()
            sendToCar.joystick = joystick;

        // Auto mode
        } else {
            // Use the latest data received from the PC and redirect to the CAR

            // The receiveFromPC struct is updated by readSerialFromPC()
            // in the beginning of loop()
            if (newPCDataAvailable) {
                sendToCar.joystick = receiveFromPC.joystick;
                newPCDataAvailable = false; // Consume the flag; Await for new data
            }
        }

        // === SEND PAYLOAD ===
        // Try to send data to the car until car receives it
        bool success = radio.write(&sendToCar, sizeof(sendToCar));
        if (success) {
            sending = !sending; // Switch roles
            radio.startListening();
        } else {
            radio.flush_tx();
            Serial.println("Remote failed to send payload.");
//            failedToSend++;
//
//            if (failedToSend >= failedToSendMax) {
//                sending = !sending;
//                failedToSend = 0;
//
////                receive.joystick.speed = 0;
////                receive.joystick.turn = 0;
//            }
        }

    } else {
        // Receive data to the car over radio
        // and redirect it to PC over serial

        // Start listening to any data from the car
        radio.startListening();

        // If there is anything to read
        if (radio.available()) {
            // Read from car
            radio.read(&receiveFromCar, sizeof(receiveFromCar));

            // Redirect data from car to PC
            sendToPC.lidar_updated = receiveFromCar.lidar_updated;
            sendToPC.lidar = receiveFromCar.lidar;
            sendToPC.gyro_updated = receiveFromCar.gyro_updated;
            sendToPC.gyro = receiveFromCar.gyro;
            sendToPC.timestamp = receiveFromCar.timestamp; // sensor reading timestamp

            // Read joystick data and add it to PC data
            sendToPC.joystick = joystick;
            sendToPC.mode = mode;

            // Send data to PC
            sendSerialToPC(sendToPC);

            // Switch roles
            sending = !sending;
        }
    }

    // Slow down loop()
    // Dirty fix to car and remote desync issue
    // When this happens, both devices get into receive or send states
    // and no communication happens
    // Time between two sends is two loop iteration, or 2 * LOOP_WAIT
    delay(LOOP_WAIT);
}

