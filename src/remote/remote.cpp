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
FIFOBuffer<CarToRemote, 20> carDataBuffer;

// State variables
bool sending = true;    // true by default, as it is the first to send data
bool mode = true;       // true = manual; false = auto
bool newPCDataAvailable = false;    // Flag for new command from PC

// Variables for  serial read from PC
byte serialRxBuffer[SERIAL_RX_BUFFER_SIZE];
uint8_t serialRxIndex = 0;
bool receivingSerial = false;
bool newData = false;

unsigned long lastSendTime = 0;
unsigned long sendInterval = 25;


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
    JoystickRead data = {
        // correct for joystick drift
        // dead zone values obtained empirically
        static_cast<short>(constrain(-(speed < 20 && speed > -20 ? 0 : speed), -255, 255)),
        static_cast<short>(constrain((turn < 20 && turn > -20 ? 0 : turn), -255, 255)),
        static_cast<bool>(k)
    };

    return data;
}


// Sends a structure wrapped in start/end markers over Serial
template<typename T>
void sendStructure(const T& dataToSend) {
    Serial.write(SERIAL_START_MARKER);
    Serial.write((byte*)&dataToSend, sizeof(T));
    Serial.write(SERIAL_END_MARKER);
    Serial.flush(); // Ensure data is sent before potentially blocking reads/delays
}

// Prepares the data structure and sends it to the PC
void prepareAndSendSerialData() {
    CarToRemote intermediate;
    while (!carDataBuffer.isEmpty()) {
        carDataBuffer.dequeue(intermediate);
        // 1. Read local sensors/inputs
        sendToPC.joystick = readJoystick(JOYSTICK_X_PIN, JOYSTICK_Y_PIN);
        sendToPC.timestamp = intermediate.timestamp;
        sendToPC.mode = true; // Example: Set mode (perhaps change based on PC command later)

        // 2. Fill placeholder data for sensors not present on this Arduino
        sendToPC.lidar = intermediate.lidar;       // Placeholder
        sendToPC.azimuth = intermediate.azimuth;        // Placeholder
        sendToPC.gyro = intermediate.gyro;

        // 3. Send the populated structure
        sendStructure(sendToPC);
    }
}


void updateReplyData() {
    sendToCar.joystick = readJoystick(JOYSTICK_X_PIN, JOYSTICK_Y_PIN);
    sendToCar.timestamp = millis();

    radio.writeAckPayload(1, &sendToCar, sizeof(RemoteToCar)); // load the payload for the next time
}


void getData() {
    if ( radio.available() ) {
        radio.read( &receiveFromCar, sizeof(CarToRemote) );
        updateReplyData();
        newData = true;
    }
}


int prev_angle = 0;
void processData() {
    if (newData == true) {
        // Serial.print("\nData received ");
        Serial.println(receiveFromCar.lidar.sensor1.angle - prev_angle);
        prev_angle = receiveFromCar.lidar.sensor1.angle;
        carDataBuffer.enqueue(receiveFromCar);
        // Serial.println(F("========== CarToRemote =========="));
        //
        // Serial.println(F("LIDARS:"));
        // Serial.print(F("  1 Angle   : ")); Serial.println(receiveFromCar.lidar.sensor1.angle);
        // Serial.print(F("  1 Distance: ")); Serial.println(receiveFromCar.lidar.sensor1.distance);
        // Serial.print(F("  2 Angle   : ")); Serial.println(receiveFromCar.lidar.sensor2.angle);
        // Serial.print(F("  2 Distance: ")); Serial.println(receiveFromCar.lidar.sensor2.distance);
        // Serial.print(F("  3 Angle   : ")); Serial.println(receiveFromCar.lidar.sensor3.angle);
        // Serial.print(F("  3 Distance: ")); Serial.println(receiveFromCar.lidar.sensor3.distance);
        //
        // Serial.println(F("Azimuth:"));
        // Serial.print(F("  Degrees : ")); Serial.println(receiveFromCar.azimuth, 2);
        //
        // Serial.println(F("Gyroscope & Accelerometer:"));
        // Serial.print(F("  Accel X : ")); Serial.println(receiveFromCar.gyro.accelX, 2);
        // Serial.print(F("  Accel Y : ")); Serial.println(receiveFromCar.gyro.accelY, 2);
        // Serial.print(F("  Accel Z : ")); Serial.println(receiveFromCar.gyro.accelZ, 2);
        // Serial.print(F("  Gyro  X : ")); Serial.println(receiveFromCar.gyro.gyroX, 2);
        // Serial.print(F("  Gyro  Y : ")); Serial.println(receiveFromCar.gyro.gyroY, 2);
        // Serial.print(F("  Gyro  Z : ")); Serial.println(receiveFromCar.gyro.gyroZ, 2);
        //
        // Serial.print(F("Timestamp : ")); Serial.println(receiveFromCar.timestamp);
        //
        // Serial.println(F("================================="));
        //
        // Serial.print(" ackPayload sent ");
        // Serial.print(sendToCar.joystick.speed);
        // Serial.print(" ");
        // Serial.print(sendToCar.joystick.turn);
        // Serial.println();
        newData = false;
    }
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
    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.openReadingPipe(1, readingPipe);

    radio.enableAckPayload();
    radio.startListening();
    radio.writeAckPayload(1, &sendToCar, sizeof(RemoteToCar)); // pre-load data
}

void loop() {
    getData();
    processData();

    unsigned long currentTime = millis();
    if (currentTime - lastSendTime >= sendInterval) {
        prepareAndSendSerialData();
        lastSendTime = currentTime;
    }
}

