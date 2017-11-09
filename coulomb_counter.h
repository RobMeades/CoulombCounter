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

#ifndef COULOMB_COUNTER
#define COULOMB_COUNTER

/**
 * @file coulomb_counter.h
 */

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/** Device I2C address. */
#define BATTERY_GAUGE_BQ35100_ADDRESS 0x55

/** I2C clock frequency.
 * NOTE: the battery shield board on the C030 platform will not work
 * at the default I2C clock frequency.
 */
#define I2C_CLOCK_FREQUENCY 250

/** Settling time after gaugeEnable is set high */
#define GAUGE_ENABLE_SETTLING_TIME_MS 10

/* ----------------------------------------------------------------
 * CLASSES
 * -------------------------------------------------------------- */

/** Coulomb counter driver (for BQ35100). */
class CoulombCounter {
public:

    /** Constructor. */
    CoulombCounter(void);
    /** Destructor. */
    ~CoulombCounter(void);

    /** Initialise the Coulomb counter chip.  Once initialised
    * the chip is put into its lowest power state.  Any API call
    * will awaken the chip from this state and then return it once
    * more to the lowest possible power state.
    * @param pI2c a pointer to the I2C instance to use.
    * @param gaugeEnable the gauge enable pin (will be set high to enable the chip).
    * @param address 7-bit I2C address of the battery gauge chip.
    * @return true if successful, otherwise false.
    */
    bool init(I2C * pI2c, PinName gaugeEnable, uint8_t address = BATTERY_GAUGE_BQ35100_ADDRESS);
    
    /** Switch on the battery gauge.  Battery gauging must be switched on
    * for the coulomb count to be valid. The chip will consume more when
    * gauging is switched on.
    * @return true if successful, otherwise false.
    */
    bool enableGauge(void);

    /** Switch off coulomb counting.
    * @return true if successful, otherwise false.
    */
    bool disableGauge(void);

    /** Check whether coulomb counting is enabled or not.
    * @return true if coulomb counting is enabled, otherwise false.
    */
    bool isGaugeEnabled(void);

    /** Read the temperature of the coulomb counter chip.
    * @param pTemperatureC place to put the temperature reading.
    * @return true if successful, otherwise false.
    */
    bool getTemperature(int32_t *pTemperatureC);

    /** Read the voltage of the battery.
    * @param pVoltageMV place to put the voltage reading.
    * @return true if successful, otherwise false.
    */
    bool getVoltage(int32_t *pVoltageMV);

    /** Read the current flowing from the battery.
    * @param pCurrentMA place to put the current reading.
    * @return true if successful, otherwise false.
    */
    bool getCurrent(int32_t *pCurrentMA);

    /** Read the battery capacity used in uAh.
    * @param pCapacityUAh place to put the capacity reading.
    * @return true if successful, otherwise false.
    */
    bool getUsedCapacity(uint32_t *pCapacityUAh);

protected:
    /** Pointer to the I2C interface. */
    I2C * gpI2c;
    /** The address of the device. */
    uint8_t gAddress;
    /** The gauge enable pin. */
    DigitalOut *pGaugeEnable;
    /** Flag to indicate device is ready. */
    bool gReady;
    /** Flag to indicate that monitor mode is active. */
    bool gGaugeOn;
    
    /** Read two bytes starting at a given address.
     * Note: gpI2c should be locked before this is called.
     * @param registerAddress the register address to start reading from.
     * @param pBytes place to put the two bytes.
     * @return true if successful, otherwise false.
     */
    bool getTwoBytes(uint8_t registerAddress, uint16_t *pBytes);
    
    /** Make sure that the chip is awake and has taken a reading.
    * Note: the function does its own locking of gpI2C so that it isn't
    * held for the entire time we wait for ADC readings to complete.
    * @return true if successful, otherwise false.
    */
    bool makeAdcReading(void);
};

#endif

/* End Of File */
