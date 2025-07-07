# DHT-read

Reading temperature and humidity data from a DHT22 (or DHT11 - but those aren't really good) using just the standard Linux libgpiod library.

## Background

I wrote this in C to make sure we have the minimal number of dependencies - this also makes it straight forward to create a static binary for the little application.

I developed and tested the code on Raspberry Pi4 and Pi5 boards but see no reason why it wouldn't work on others Single Board Computers.

The code is a rather crude implementation of the logic described in the DHT11 and DHT22 datasheets
https://components101.com/sites/default/files/component_datasheet/DFR0067%20DHT11%20Datasheet.pdf
https://components101.com/sites/default/files/component_datasheet/DHT22%20Sensor%20Datasheet.pdf

Note that the 'integral' and 'decimal' data description in the DHT22 datasheet appears to be nonsense. Instead this is a 16bit integer value (MSB first) that gives 10 times the temperature (in ºC) and humidity percentage.

This project was inspired by a number of other projects, most importantly I guess https://github.com/jeffep/raspi5-DHT22 -- but I ended up fundamentally disliking how this code did a fixed lookback approach to determining bit values. Timing instabilities seemed to make it extremely hard to make this work across different Raspberry Pi boards. Instead I completely rewrote the code to simply analyse how many samples we are reading for the reference "low" value and then derive from that a threshold for the duration of the "high" value. This is explained in more detail in the DHT.c file.

Since I started out with the code from that project (even though I translated it to C (who needs C++ for something silly like this?) and then ended up essentially rewriting all of it) I still want to point out that in an abundance of deference to that inspiration I decided to put this code under the same license:

// SPDX-License-Identifier: AGPL-3.0


## How to build

All you need is a C compiler and the `libgpiod-dev` package

gcc --static -o dht src/main.c src/DHT.c -Iinclude -lgpiod

## Usage

`./dht [-d] [-j] [-t]`

Without arguments it will print out temperature and humidity detected, and how many attempts it took to get those from the sensor. On the Pi5 this almost always worked with just one round of reading data, but on the Pi4 it usually took three to four rounds of reading data and checking that it was valid. Since the recommendation is to wait a while between attempts to read from the sensor, this means that it can take quite a while for the app to respond.

`-d` provides debug output, including all of the raw data
`-j` provides the output as json
`-t` results in only the temperature as a simple float
