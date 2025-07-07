// SPDX-License-Identifier: AGPL-3.0

#ifndef DHT_H
#define DHT_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h> // for size_t
#include <unistd.h> // for usleep
typedef struct {
    struct gpiod_chip *chip;
    struct gpiod_line *line;
    int debugMode;
} DHT;

// Initialize the DHT sensor on the given GPIO pin. Returns pointer or NULL on failure.
DHT* DHT_init(int dhtPin, int debug);

// Free the DHT sensor resources.
void DHT_free(DHT* sensor);

// Send start signal and sample data. Fills rawData array of length rawDataLen.
bool DHT_read_data(DHT* sensor, int* rawData, size_t rawDataLen);

// Decode the sampled data into humidity and temperature.
bool DHT_decode_data(DHT* sensor, const int* rawData, size_t rawDataLen, float* humidity, float* temperature);

#endif // DHT_H
