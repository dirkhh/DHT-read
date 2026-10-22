# DHT-read

Simple native binary to read DHT11/22 sensor values on Raspberry Pi.

v1.0.0:
- decoder no longer depends on where in the sensor's signal sampling starts - it decodes the last 40 bits
  it sees, so the response signal at the start no longer confuses it
- switch the pin from output to input with a single call, so sampling starts sooner and with less jitter
- these should reduce the number of retries needed, especially on slower boards (confirmed working on RPi3)

v0.1.0:
- first working version, tested on RPi4b and RPi5
- first attempt to get GitHub workflow to create binary
