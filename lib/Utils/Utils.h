//
// Created by Kadir on 09.04.2025.
//

#ifndef UTILS_H
#define UTILS_H

#include <RF24.h>
#include <stddef.h>

bool setupRadio(RF24& radio, byte readingPipe[6], byte writingPipe[6]);

bool checkRadio(RF24& radio, bool initiator, int timeout, int tries);

template <typename T, size_t Size>
class FIFOBuffer {
private:
    T buffer[Size];
    size_t head = 0;
    size_t tail = 0;
    size_t count = 0;

public:
    // Check if buffer is full
    bool isFull() const {
        return count == Size;
    }

    // Check if buffer is empty
    bool isEmpty() const {
        return count == 0;
    }

    // Add item to buffer
    bool enqueue(const T& item) {
        if (isFull()) {
            return false;
        }
        buffer[head] = item;
        head = (head + 1) % Size;
        count++;
        return true;
    }

    // Remove oldest item from buffer
    bool dequeue(T& item) {
        if (isEmpty()) {
            return false;
        }
        item = buffer[tail];
        tail = (tail + 1) % Size;
        count--;
        return true;
    }

    // Peek at the oldest item without removing it
    bool peek(T& item) const {
        if (isEmpty()) {
            return false;
        }
        item = buffer[tail];
        return true;
    }

    // Get current number of items
    size_t size() const {
        return count;
    }

    // Clear the buffer
    void clear() {
        head = 0;
        tail = 0;
        count = 0;
    }
};

#endif
