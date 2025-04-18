//
// Created by Kadir on 09.04.2025.
//

#include "Utils.h"
#include <RF24.h>
#include <Arduino.h>
// #include <structs.h>


// Function to setup NRF24 module.
// Takes a reference to radio and pipe addresses
// Returns true if setuped successfully, false otherwise
bool setupRadio(RF24& radio,  byte readingPipe[6], byte writingPipe[6]) {
    // Try to initialize the radio
    if(!radio.begin()) {
        Serial.println("Failed to initialize radio");
        return false;
    }

    // Check if a radio module connected to arduino
    if (radio.isChipConnected()) {
        Serial.println("RF24 Chip connected");
    } else {
        Serial.println("RF24 chip not connected");
        return false;
    }

    // Set power amplifier level to low (safer for modules without external power)
    // radio.setPALevel(RF24_PA_MAX);
    radio.setPALevel(RF24_PA_MIN);
    radio.setDataRate(RF24_250KBPS);
    // radio.setRetries(5,15);
    radio.setChannel(124);

    // Set pipe addresses
    radio.openWritingPipe(writingPipe);
    radio.openReadingPipe(1, readingPipe);

    return true;
}


bool checkRadio(RF24& radio, bool initiator, int timeout, int tries) {
    const char ackPayload[12] = "Knock knock";
    char receiveBuffer[12] = {0};

    if (initiator) {
        int counter = 0;
        radio.stopListening();
        while(!radio.write(ackPayload, sizeof(ackPayload)) && counter < tries) {
            delay(10);
            counter++;
        }

        radio.startListening();
        int start = millis();
        while (millis() - start < timeout) {
            if (radio.available()) {
                radio.read(&receiveBuffer, sizeof(receiveBuffer));
                return true;
            }
        }
        return false;
    } else {
        radio.startListening();
        int start = millis();
        while (millis() - start < timeout) {
            if (radio.available()) {
                radio.read(&receiveBuffer, sizeof(receiveBuffer));
                break;
            }
        }

        int counter = 0;
        radio.stopListening();
        while(!radio.write(ackPayload, sizeof(ackPayload)) && counter < tries) {
            delay(10);
            counter++;
        }

        if (counter >= tries || millis() - start > timeout) {
            return false;
        }
    }
}


// template <typename T, size_t Size>
// class FIFOBuffer {
// private:
//     T buffer[Size];
//     size_t head = 0;
//     size_t tail = 0;
//     size_t count = 0;
//
// public:
//     bool isFull() const {
//         return count == Size;
//     }
//
//     bool isEmpty() const {
//         return count == 0;
//     }
//
//     bool enqueue(const T& item) {
//         if (isFull()) {
//             return false;
//         }
//         buffer[head] = item;
//         head = (head + 1) % Size;
//         count++;
//         return true;
//     }
//
//     bool dequeue(T& item) {
//         if (isEmpty()) {
//             return false;
//         }
//         item = buffer[tail];
//         tail = (tail + 1) % Size;
//         count--;
//         return true;
//     }
// }
