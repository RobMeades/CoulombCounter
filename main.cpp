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

#include "mbed.h"
#include "coulomb_counter.h"

// LEDs
DigitalOut ledRed(LED1, 1);
DigitalOut ledGreen(LED2, 1);


int main()
{
    I2C i2C(I2C_SDA, I2C_SCL);
    CoulombCounter coulombCounter;
    uint32_t coulombsUsed;

    printf ("Starting up...\n");
    if (coulombCounter.init(&i2C, D4)) {        
        printf ("Coulomb Counter initialised.\n");
        if (coulombCounter.enableGauge()) {
            while (1) {
                if (coulombCounter.getUsedCapacity(&coulombsUsed)) {
                    printf ("Energy used: %.3f mAh.\n", (float) coulombsUsed / 1000);
                } else {
                    printf ("Unable to read coulomb counter.\n");
                }
                wait_ms(1000);
            }
        } else {
            printf ("Unable to initialise coulomb counter.\n");
        }
    } else {
        printf ("Unable to initialise coulomb counter.\n");
    }
    
    ledGreen = 1;
    ledRed = 0;
    printf("Should never get here.\n");
    MBED_ASSERT(false);
}

// End Of File