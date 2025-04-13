//
// Created by Kadir on 09.04.2025.
//

#ifndef UTILS_H
#define UTILS_H

#include <RF24.h>

bool setupRadio(RF24& radio, byte readingPipe[6], byte writingPipe[6]);

bool checkRadio(RF24& radio, bool initiator, int timeout, int tries);

#endif
