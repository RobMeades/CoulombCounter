/* mbed Microcontroller Library
 * Copyright (c) 2017 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file coulombs_counter.cpp
 */

//#define DEBUG_COULOMB_COUNTER

#include <mbed.h>
#include <coulomb_counter.h>

#ifdef DEBUG_COULOMB_COUNTER
# include <stdio.h>
#endif

// ----------------------------------------------------------------
// COMPILE-TIME MACROS
// ----------------------------------------------------------------

/** How long to wait for each loop while device is initialising. */
#define INIT_LOOP_WAIT_MS 100

/** The maximum number of init loops to wait for. */
#define INIT_LOOP_COUNT 10

// ----------------------------------------------------------------
// PRIVATE VARIABLES
// ----------------------------------------------------------------

// ----------------------------------------------------------------
// GENERIC PRIVATE FUNCTIONS
// ----------------------------------------------------------------

// Read two bytes from an address.
// Note: gpI2c should be locked before this is called.
bool CoulombCounter::getTwoBytes (uint8_t registerAddress, uint16_t *pBytes)
{
    bool success = false;
    char data[3];

    if (gpI2c != NULL) {
        data[0] = registerAddress;
        data[1] = 0;
        data[2] = 0;

        // Send a command to read from registerAddress
        if ((gpI2c->write(gAddress, &(data[0]), 1, true) == 0) &&
            (gpI2c->read(gAddress, &(data[1]), 2) == 0)) {
            success = true;
            if (pBytes) {
                *pBytes = (((uint16_t) data[2]) << 8) + data[1];
            }
        }
    }

    return success;
}

// Make sure that the device is awake and has taken a reading.
// Note: the function does its own locking of gpI2C so that it isn't
// locked for the entire time we wait for ADC readings to complete.
bool CoulombCounter::makeAdcReading(void)
{
    bool success = false;
    uint16_t controlStatus;
    char data[1];
    
    // Wait for INITCOMP to be set
    data[0] = 0;  // Set address to first register for Control

    gpI2c->lock();
    // Raise the pin
    *pGaugeEnable = 1;    
    wait_ms(GAUGE_ENABLE_SETTLING_TIME_MS);
    for (int x = 0; !success && (x < INIT_LOOP_COUNT); x++) {
        if (gpI2c->write(gAddress, &(data[0]), 1) == 0) {
            if (getTwoBytes(0, &controlStatus)) {
                // Bit 7 is INITCOMP
                if (((controlStatus >> 7) & 0x01) == 0x01) {
                    success = true;
                }
            }
            wait_ms (INIT_LOOP_WAIT_MS);
        }
    }
    gpI2c->unlock();
    
    return success;
}

//----------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------

// Constructor.
CoulombCounter::CoulombCounter(void)
{
    gpI2c = NULL;
    pGaugeEnable = NULL;
    gReady = false;
    gGaugeOn = false;
}

// Destructor.
CoulombCounter::~CoulombCounter(void)
{
}

// Initialise ourselves.
bool CoulombCounter::init(I2C * pI2c, PinName gaugeEnable, uint8_t address)
{
    uint16_t answer;
    char data[8];

    gpI2c = pI2c;
    gAddress = address << 1;
    
    pGaugeEnable = new DigitalOut(gaugeEnable, 1);
    wait_ms(GAUGE_ENABLE_SETTLING_TIME_MS);

    if (gpI2c != NULL) {
        gpI2c->lock();
        gpI2c->frequency(I2C_CLOCK_FREQUENCY);
        
        // Send a control command to read the device type
        data[0] = 0x3e;  // Set address to ManufacturerAccessControl
        data[1] = 0x03;  // First byte of HW_VERSION sub-command (0x03)
        data[2] = 0x00;  // Second byte of HW_VERSION sub-command (0x00) (register address will auto-increment)

        if ((gpI2c->write(gAddress, &(data[0]), 3) == 0) &&
            getTwoBytes(0x40, &answer)) {  // Read from MACData address
            if (answer == 0x00a8) {
                gReady = true;
            }

#ifdef DEBUG_COULOMB_COUNTER
            printf("CoulombCounter (I2C 0x%02x): read 0x%04x as HW_VERSION, expected 0x00a8.\n", gAddress >> 1, answer);
#endif
        }

        if (!gGaugeOn) {
            *pGaugeEnable = 0;
        }
                
        gpI2c->unlock();
    }

#ifdef DEBUG_COULOMB_COUNTER
    if (gReady) {
        printf("CoulombCounter (I2C 0x%02x): handler initialised.\r\n", gAddress >> 1);
    } else {
        printf("CoulombCounter (I2C 0x%02x): init NOT successful.\r\n", gAddress >> 1);
    }
#endif

    return gReady;
}

// Switch on the battery capacity monitor.
bool CoulombCounter::enableGauge(void)
{
    bool success = false;
    
    if (gReady) {
        *pGaugeEnable = 1;
        wait_ms(GAUGE_ENABLE_SETTLING_TIME_MS);
        success = true;
    }
    
    return success;
}

// Switch off the battery capacity monitor.
bool CoulombCounter::disableGauge(void)
{
    bool success = false;
    
    if (gReady) {
        *pGaugeEnable = 0;
        success = true;
    }
    
    return success;
}

// Determine whether battery gauging is enabled.
bool CoulombCounter::isGaugeEnabled(void)
{
    bool isEnabled = false;
    
    if (gReady) {
        isEnabled = true;
    }
    
    return isEnabled;
}


// Get the temperature of the chip.
bool CoulombCounter::getTemperature(int32_t *pTemperatureC)
{
    bool success = false;
    int32_t temperatureC = 0;
    uint16_t data;

    if (gReady && (gGaugeOn || makeAdcReading())) {
        gpI2c->lock();
        // Read from the temperature register address
        if (getTwoBytes (0x06, &data)) {
            success = true;

            // The answer is in units of 0.1 K, so convert to C
            temperatureC = ((int32_t) data / 10) - 273;

            if (pTemperatureC) {
                *pTemperatureC = temperatureC;
            }

#ifdef DEBUG_COULOMB_COUNTER
            printf("CoulombCounter (I2C 0x%02x): chip temperature %.1f K, so %d C.\n", gAddress >> 1, ((float) data) / 10, (int) temperatureC);
#endif
        }
        
        if (!gGaugeOn) {
            *pGaugeEnable = 0;
        }
        
        gpI2c->unlock();
    }

    return success;
}

// Get the voltage of the battery.
bool CoulombCounter::getVoltage(int32_t *pVoltageMV)
{
    bool success = false;
    uint16_t data = 0;

    if (gReady && (gGaugeOn || makeAdcReading())) {
        gpI2c->lock();
        // Read from the voltage register address
        if (getTwoBytes (0x08, &data)) {
            success = true;

            // The answer is in mV
            if (pVoltageMV) {
                *pVoltageMV = (int32_t) data;
            }

#ifdef DEBUG_COULOMB_COUNTER
            printf("CoulombCounter (I2C 0x%02x): battery voltage %.3f V.\n", gAddress >> 1, ((float) data) / 1000);
#endif
        }

        if (!gGaugeOn) {
            *pGaugeEnable = 0;
        }
        
        gpI2c->unlock();
    }
    
    return success;
}

// Get the current flowing from the battery.
bool CoulombCounter::getCurrent(int32_t *pCurrentMA)
{
    bool success = false;
    int32_t currentMA = 0;
    uint16_t data;

    if (gReady && (gGaugeOn || makeAdcReading())) {
        gpI2c->lock();            
        // Read from the average current register address
        if (getTwoBytes (0x0c, &data)) {
            success = true;

            if (pCurrentMA) {
                *pCurrentMA = currentMA;
            }

#ifdef DEBUG_COULOMB_COUNTER
            printf("CoulombCounter (I2C 0x%02x): current %d mA.\n", gAddress >> 1, (int) currentMA);
#endif
        }

        if (!gGaugeOn) {
            *pGaugeEnable = 0;
        }
        
        gpI2c->unlock();
    }
    
    return success;
}

// Get the battery capacity used.
bool CoulombCounter::getUsedCapacity(uint32_t *pCapacityUAh)
{
    bool success = false;
    char bytes[5];
    uint32_t data;

    if (gReady && (gGaugeOn || makeAdcReading())) {
        gpI2c->lock();
        // Read four bytes from the AccummulatedCapacity register address
        
        // Send a command to read from registerAddress
        bytes[0] = 0x02;
        bytes[1] = 0;
        bytes[2] = 0;
        bytes[3] = 0;
        bytes[4] = 0;

        if ((gpI2c->write(gAddress, &(bytes[0]), 1) == 0) &&
            (gpI2c->read(gAddress, &(bytes[1]), 4) == 0)) {
            success = true;
            data = (((uint32_t) bytes[4]) << 24) + (((uint32_t) bytes[3]) << 16) + (((uint32_t) bytes[2]) << 8) + bytes[1];
 
            // The answer is in uAh
            if (pCapacityUAh) {
                *pCapacityUAh = data;
            }

#ifdef DEBUG_COULOMB_COUNTER
            printf("CoulombCounter (I2C 0x%02x): energy used %u uAh.\n", gAddress >> 1, (unsigned int) data);
#endif
        }

        if (!gGaugeOn) {
            *pGaugeEnable = 0;
        }
        
        gpI2c->unlock();
    }
    
    return success;
}

/* End Of File */
