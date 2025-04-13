#include <RF24.h> // Include RF24 library for easier control
#include <SPI.h> // Include SPI library for communication with nRF24
#include <nRF24L01.h> // Include nRF24L01 hardware definitions

RF24 radio(7, 8); // Create RF24 object with CE pin 7, CSN pin 8

const byte addressA[6] = "00001"; // Define address A for one radio
const byte addressB[6] = "00002"; // Define address B for the other radio
char incomingMessage[32] = {0}; // Buffer to store incoming messages

bool isFirst = false; // <- Set to false on the second Arduino to swap roles

void setup() {
    Serial.begin(19200); // Initialize Serial communication at 9600 baud
    radio.begin(); // Initialize the radio module
    radio.setPALevel(RF24_PA_LOW); // Set power amplifier level to low (safer for modules without external power)

    if (isFirst) {
        radio.openWritingPipe(addressB); // If this is the first Arduino, set writing pipe to address B
        radio.openReadingPipe(1, addressA); // Set reading pipe to listen on address A
    } else {
        radio.openWritingPipe(addressA); // If this is the second Arduino, set writing pipe to address A
        radio.openReadingPipe(1, addressB); // Set reading pipe to listen on address B
    }

    radio.startListening(); // Start listening for incoming messages
    Serial.println("Radio Chat Ready. Type your message:"); // Print ready message to Serial Monitor
}

void loop() {
    // Check if there is incoming data
    if (radio.available()) {
        radio.read(&incomingMessage, sizeof(incomingMessage)); // Read the incoming message into the buffer
        Serial.print("Friend: "); // Print prefix to Serial Monitor
        Serial.println(incomingMessage); // Print the received message
        memset(incomingMessage, 0, sizeof(incomingMessage)); // Clear the buffer after reading
    }

    // Check if user typed something in the Serial Monitor
    if (Serial.available()) {
        String outgoingMessage = Serial.readStringUntil('\n'); // Read the message until newline character
        outgoingMessage.trim(); // Remove any trailing whitespace or newline

        if (outgoingMessage.length() > 0) {
            radio.stopListening(); // Stop listening to send a message
            radio.write(outgoingMessage.c_str(), outgoingMessage.length() + 1); // Send the message over the radio
            radio.startListening(); // Resume listening mode after sending
        }
    }
}
