/*
  DHT22.cpp - DHT22 sensor library
  Developed by Ben Adams - 2011

  Modified by Jouni Seppanen - 2014

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


Humidity and Temperature Sensor DHT22 info found at
http://www.sparkfun.com/products/10167

Version 0.6: 28 Nov 2014 by Jouni Seppanen
- Memory usage optimization
- Dropped support for Fahrenheit

Version 0.5: 15 Jan 2012 by Craig Ringer
- Updated to build against Arduino 1.0
- Made accessors inline in the header so they can be optimized away

Version 0.4: 24-Jan-2011 by Ben Adams
Added return code constants to keywords.txt
Returns DHT_ERROR_CHECKSUM on check sum mismatch 

Version 0.3: 17-Jan-2011 by Ben Adams
This version reads data
Needs check sum code added at the end of readData

Version 0.2: 16-Jan-2011 by Ben Adams
Changed coding style to match other Arduino libraries.
This version will not read data either!

Version 0.1: 10-Jan-2011 by Ben Adams nethoncho AT gmail.com
First Version is a skeleton. This version will not read data!
Code adapted from the following sources:
The Arduino OneWire lib
http://sheepdogguides.com/arduino/ar3ne1humDHT11.htm

*/

#include "DHT22.h"
#include <Arduino.h>
#include <pins_arduino.h>

extern "C" {
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
}

#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*(base+1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*(base+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*(base+2)) &= ~(mask))

// This should be 40, but the sensor is adding an extra bit at the start
#define DHT22_DATA_BIT_COUNT 41

inline uint16_t Convert(uint8_t const * const data, uint8_t const numOfBits) {
    uint8_t bit;
    uint8_t startIdx = numOfBits - 1;
    uint16_t result = 0;
    
    for(bit = 0; bit < numOfBits; bit++) {
        if(data[bit + 1] > 11) {
            result |= (1 << (startIdx - bit));
        }
    }
    return result;
}


DHT22::DHT22(uint8_t const pin)
{
    _bitmask         = digitalPinToBitMask(pin);
    _baseReg         = portInputRegister(digitalPinToPort(pin));
    _lastHumidity    = DHT22_ERROR_VALUE;
    _lastTemperature = DHT22_ERROR_VALUE;
}

//
// Read the 40 bit data stream from the DHT 22
// Store the results in private member data to be read by public member functions
//
DHT22_ERROR_t DHT22::readData()
{
    uint8_t bitmask                  = _bitmask;
    volatile uint8_t *reg asm("r30") = _baseReg;
    uint8_t retryCount;
    uint8_t bitTimes[DHT22_DATA_BIT_COUNT];
    int currentHumidity    = 0;
    int currentTemperature = 0;
    uint8_t checkSum       = 0;
    uint8_t checkSumCalc   = 0;
    uint8_t bit;

    // Pin needs to start HIGH, wait until it is HIGH with a timeout
    cli();
    DIRECT_MODE_INPUT(reg, bitmask);
    sei();
    retryCount = 0;
    do {
        if (retryCount > 125) {
            return DHT_BUS_HUNG;
        }
        retryCount++;
        delayMicroseconds(2);
    } while(!DIRECT_READ(reg, bitmask));
    
    // Send the activate pulse
    cli();
    DIRECT_WRITE_LOW(reg, bitmask);
    DIRECT_MODE_OUTPUT(reg, bitmask); // Output Low
    sei();
    delayMicroseconds(1100); // 1.1 ms
    cli();
    DIRECT_MODE_INPUT(reg, bitmask);	// Switch back to input so pin can float
    sei();
    // Find the start of the ACK Pulse
    retryCount = 0;
    do {
        if (retryCount > 25) { //(Spec is 20 to 40 us, 25*2 == 50 us)
            return DHT_ERROR_NOT_PRESENT;
        }
        retryCount++;
        delayMicroseconds(2);
    } while(!DIRECT_READ(reg, bitmask));

    // Find the end of the ACK Pulse
    retryCount = 0;
    do {
        if (retryCount > 50) { //(Spec is 80 us, 50*2 == 100 us)
            return DHT_ERROR_ACK_TOO_LONG;
        }
        retryCount++;
        delayMicroseconds(2);
    } while(DIRECT_READ(reg, bitmask));

    // Read the 40 bit data stream
    for(bit = 0; bit < DHT22_DATA_BIT_COUNT; bit++) {
        // Find the start of the sync pulse
        retryCount = 0;
        do {
            if (retryCount > 35) { //(Spec is 50 us, 35*2 == 70 us)
                return DHT_ERROR_SYNC_TIMEOUT;
            }
            retryCount++;
            delayMicroseconds(2);
        } while(!DIRECT_READ(reg, bitmask));

        // Measure the width of the data pulse
        retryCount = 0;
        do {
            if (retryCount > 50) { //(Spec is 80 us, 50*2 == 100 us)
                return DHT_ERROR_DATA_TIMEOUT;
            }
            retryCount++;
            delayMicroseconds(2);
        } while(DIRECT_READ(reg, bitmask));
        bitTimes[bit] = retryCount;
    }
    // Now bitTimes have the number of retries (us *2)
    // that were needed to find the end of each data bit
    // Spec: 0 is 26 to 28 us
    // Spec: 1 is 70 us
    // bitTimes[x] <= 11 is a 0
    // bitTimes[x] >  11 is a 1
    // Note: the bits are offset by one from the data sheet, not sure why
    currentHumidity    = Convert(&bitTimes[1], 16);
    currentTemperature = Convert(&bitTimes[17], 16);
    checkSum           = Convert(&bitTimes[33], 8);

    _lastHumidity = currentHumidity & 0x7FFF;
    if(currentTemperature & 0x8000) {
        // Below zero, non standard way of encoding negative numbers!
        // Convert to native negative format.
        _lastTemperature = 0 - (currentTemperature & 0x7FFF);
    } else {
        _lastTemperature = currentTemperature;
    }

    checkSumCalc += currentHumidity >> 8;
    checkSumCalc += currentHumidity & 0xFF;
    checkSumCalc += currentTemperature >> 8;
    checkSumCalc += currentTemperature & 0xFF;
    if(checkSum != checkSumCalc) {
        return DHT_ERROR_CHECKSUM;
    }
    return DHT_ERROR_NONE;
}
