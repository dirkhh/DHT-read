// SPDX-License-Identifier: AGPL-3.0

#include "DHT.h"
#include <time.h>
#include <gpiod.h>

// this does a rather crude implementation of the logic described in the DHT11 and DHT22 datasheets

// it was inspired by a number of projects, including https://github.com/jeffep/raspi5-DHT22/
// but is unlikely to be considered derived from that


// Fake OO code - a bit over the top, but whatever
// returns a pointer to a DHT structure or NULL if init fails
DHT* DHT_init(int dhtPin, int debug) {
    if (debug)
        fprintf(stderr, "[INFO] Initializing DHT on GPIO %d\n", dhtPin);

    DHT* sensor = (DHT*)malloc(sizeof(DHT));
    if (!sensor) {
        fprintf(stderr, "[ERROR] Failed to allocate memory for DHT structure\n");
        return NULL;
    }

    sensor->debugMode = debug;
    sensor->chip = gpiod_chip_open("/dev/gpiochip0");
    if (!sensor->chip) {
        fprintf(stderr, "[ERROR] Failed to open GPIO chip\n");
        free(sensor);
        return NULL;
    }

    sensor->line = gpiod_chip_get_line(sensor->chip, dhtPin);
    if (!sensor->line) {
        fprintf(stderr, "[ERROR] Failed to get GPIO line\n");
        gpiod_chip_close(sensor->chip);
        free(sensor);
        return NULL;
    }

    if (debug) {
        fprintf(stderr, "[INFO] Successfully initialized GPIO %d\n", dhtPin);
    }
    return sensor;
}

// closes and frees a DHT structure
void DHT_free(DHT* sensor) {
    if (!sensor)
        return;
    if (sensor->chip) {
        gpiod_chip_close(sensor->chip);
        if (sensor->debugMode) {
            fprintf(stderr, "[INFO] GPIO chip closed\n");
        }
    }
    free(sensor);
}

// read data from the gpio port in a tight loop
// we should be able to sleep, but that seems to really mess up the accuracy, so instead
// just go full tilt and read as fast as we can -- not ideal
bool DHT_read_data(DHT* sensor, int* rawData, size_t rawDataLen) {
    if (!sensor)
        return false;
    if (sensor->debugMode) {
        fprintf(stderr, "[INFO] Initialize GPIO in order to read from the sensor...\n");
    }
    struct gpiod_line_request_config config = {0};
    config.consumer = "DHT";
    config.request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;

    if (gpiod_line_request(sensor->line, &config, 0) < 0) {
        fprintf(stderr, "[ERROR] Failed to request GPIO line as output\n");
        return false;
    }

    // pull the data pin to low and wait for more than 1ms (1500us seems to work)
    // pull the data pin to high and wait for 30ms and then release the pin
    gpiod_line_set_value(sensor->line, 0);
    usleep(1500);
    gpiod_line_set_value(sensor->line, 1);
    usleep(30);

    gpiod_line_release(sensor->line);

    config.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
    if (gpiod_line_request(sensor->line, &config, 0) < 0) {
        fprintf(stderr, "[ERROR] Failed to request GPIO line as input\n");
        return false;
    }

    // the documentation tells us to wait for 40us but that makes the code
    // miss the first edge. instead we just start reading
    // I hate that this is a tight loop, but even the shortest sleep made
    // this fail
    for (size_t i = 0; i < rawDataLen; ++i) {
        int val = gpiod_line_get_value(sensor->line);
        if (val < 0) {
            fprintf(stderr, "[ERROR] Failed to read GPIO value at index %zu\n", i);
            break;
        }
        rawData[i] = val;
    }
    gpiod_line_release(sensor->line);

    return true;
}

// the data format specifies that each bit is sent with a 50us low voltage level,
// followed by a high voltage level that determines the bit value:
//    26-28us indicates a 0 bit
//     ~70us  indicates a 1 bit
// because we read as fast as we can instead of trying to time our reads, we need to
// analyze the data. We look at the series of 0 (low) values that we have in the data
// stream and determine the shortest and longest of those and use the average of that
// for the threshold
// when counting the 1 (high) values we simply assume that fewer than threshold implies
// a 0 bit and more than threshold a 1 bit. This is of course brutally coarse but in my
// testing the values weren't nearly as stable as I would have preferred (because really,
// looking at the specs you'd say 50% of threshold for 0 and 150% of threshold for 1).
// thankfully the data has a simplistic checksum, so we can try again if we mis-decode things.
bool DHT_decode_data(DHT* sensor, const int* rawData, size_t rawDataLen, float* humidity, float* temperature) {
    int lastData = rawData[0];
    int lastidx = -1;
    int low = rawDataLen;
    int high = 0;
    int bits[40];
    int bit = 0;
    bool seen_first_edge = false;
    if (sensor && sensor->debugMode)
        fprintf(stderr, "Raw Data(%zu)=\n", rawDataLen);
    for (size_t i = 0; i < rawDataLen; ++i) {
        if (sensor && sensor->debugMode)
            fprintf(stderr, "%d", rawData[i]);
        if (rawData[i] != lastData) { // we have an edge
            int l = i - lastidx; // this is how many low or high values we have seen since the last edge
            if (sensor && sensor->debugMode)
                fprintf(stderr, "   %d\n", l);
            if (seen_first_edge) {
                if (lastData == 0) { // if this was low values we include it in the threshold calculation
                    low = l < low ? l : low;
                    high = l > high ? l : high;
                } else { // if these were high values, we store the number to later calculate the bit value
                    bits[bit] = l;
                    bit++;
                    if (bit > 39)
                        break;
                }
            } else {
                seen_first_edge = true;
            }
            lastidx = i;
            lastData = rawData[i];
        }
    }
    if (bit < 40) {
        fprintf(stderr, "[ERROR] only detected %d bits, giving up\n", bit);
        return false;
    }
    int threshold = (low + high) / 2;
    if (sensor && sensor->debugMode) {
        fprintf(stderr, "\nthreshold %d\n", threshold);
        fprintf(stderr, "bit durations: ");
        for (int i = 0; i < 40; ++i) {
            fprintf(stderr, "%d ", bits[i]);
        }
        fprintf(stderr, "\n");
    }

    // simply add the bits together
    int data[5] = {0};
    for (int i = 0; i < 40; ++i) {
        data[i / 8] <<= 1;
        if (bits[i] > threshold) {
            data[i / 8] |= 1;
        }
    }
    // finally check the crude checksum
    int checksum = data[0] + data[1] + data[2] + data[3];
    if ((checksum & 0xFF) != data[4]) {
        fprintf(stderr, "[ERROR] Checksum mismatch: %d!=%d\n", checksum & 0xFF, data[4]);
        fprintf(stderr, "data =%d,%d,%d,%d\n", data[0], data[1], data[2], data[3]);
        return false;
    }

    // check if this is a DHT11 or DHT22
    // DHT11 has temperature in whole degrees in data[2] and humidity percentage in data[0]
    // DHT22 has temperature in tenths of degrees in data[2-3] and humidity in tenths of percent in data[0-1]
    // the logic to check here is really crude... I assume that the humidity anywhere is at least 4%, and it's
    // obviously at most 100% - if we have two byte values (DHT22) that means the most significant byte is at
    // most 3 (0x03E8 == 1000 which corresponds to 100%) - so larger values than that imply a DHT22
    float h, t;
    if (data[0] < 4) {
        h = ((float)((data[0] << 8) | data[1])) / 10.0f;
        t = ((float)((data[2] << 8) | data[3])) / 10.0f;
    } else {
        h = (float)data[0];
        t = (float)data[2];
    }
    if (humidity)
        *humidity = h;
    if (temperature)
        *temperature = t;
    return true;
}

