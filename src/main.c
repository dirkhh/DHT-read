#include "DHT.h"

int main(int argc, char* argv[]) {
    int gpioPin = 4; // Default GPIO pin
    bool debug = false;
    bool temp_only = false;
    bool json = false;

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-d") == 0) {
            debug = true;
        } else if (strcmp(argv[i], "-j") == 0) {
            json = true;
        } else if (strcmp(argv[i], "-t") == 0) {
            temp_only = true;
        } else {
            // Assume argument is a GPIO pin number
            char *endptr = NULL;
            int pin = strtol(argv[i], &endptr, 10);
            if (endptr == argv[i] || pin < 0) {
                fprintf(stderr, "[ERROR] Invalid GPIO pin number: %s\n", argv[i]);
                return 1;
            }
            gpioPin = pin;
        }
    }
    if (json && temp_only) {
        fprintf(stderr, "[ERROR] -j and -t are mutually exclusive\n");
        return 1;
    }

    if (debug) {
        fprintf(stderr,"[DEBUG] Using GPIO pin: %d\n", gpioPin);
    }

    DHT* sensor = DHT_init(gpioPin, debug);
    if (!sensor) {
        fprintf(stderr, "[ERROR] Failed to initialize DHT sensor\n");
        return 1;
    }
    bool done = false;
    int count = 0;
    int data[6000];
    float humidity = 0.0f, temperature = 0.0f;

    // try reading and decoding data from the sensor -- wait 5 seconds if that fails
    while (!done) {
        memset(data, 0, sizeof(data));
        count++;
        done = DHT_read_data(sensor, data, 6000) && DHT_decode_data(sensor, data, 6000, &humidity, &temperature);
        if (!done)
            usleep(5000000); // Sleep for 5 seconds
    }

    if (temp_only) {
        printf("%.1f\n", temperature);
    } else if (json) {
        printf("{\"humidity\": %.1f, \"temperature\": %.1f}\n", humidity, temperature);
    } else {
        printf("Humidity: %.1f %% | Temperature: %.1f°C\n", humidity, temperature);
        printf("This took %d rounds\n", count);
    }

    DHT_free(sensor);
    return 0;
}
