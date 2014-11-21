/*
 * Author: William Penner <william.penner@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 *
 * Based on code adapted from the kernel MPL3115A2 driver and
 * code by: Yevgeniy Kiveisha <yevgeniy.kiveisha@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#pragma once

#include <string>
#include <mraa/i2c.h>
#include <math.h>

#define MPL3115A2_NAME        "mpl3115a2"

#define MPL3115A2_I2C_ADDRESS 0x60

#define MPL3115A2_STATUS      0x00
#define MPL3115A2_OUT_PRESS   0x01  /* MSB first, 20 bit */
#define MPL3115A2_OUT_TEMP    0x04  /* MSB first, 12 bit */
#define MPL3115A2_WHO_AM_I    0x0c
#define MPL3115A2_PT_DATA_CFG 0x13
#define MPL3115A2_P_MIN       0x1C
#define MPL3115A2_T_MIN       0x1F
#define MPL3115A2_P_MAX       0x21
#define MPL3115A2_T_MAX       0x24
#define MPL3115A2_CTRL_REG1   0x26

#define MPL3115A2_DEVICE_ID   0xc4

#define MPL3115A2_STATUS_PRESS_RDY 0x04
#define MPL3115A2_STATUS_TEMP_RDY  0x02

#define MPL3115A2_CTRL_ACTIVE   0x01  /* continuous measurement */
#define MPL3115A2_CTRL_OST      0x02  /* initiate measurement */
#define MPL3115A2_CTRL_RESET    0x04  /* software reset */
#define MPL3115A2_CTRL_OS_258MS 0x30  /* 64x oversampling */
#define MPL3115A2_CTRL_ALT_MODE 0x80  /* Flag to set altitude mode */

#define MPL3115A2_CFG_DREM      0x04  /* Data ready event mode */
#define MPL3115A2_CFG_PDEFE     0x02  /* Pressure/Alt event flag mode */
#define MPL3115A2_CFG_TDEFE     0x01  /* Temperature event flag mode */

#define MPL3115A2_SETOVERSAMPLE(a) ((a & 7) << 3)
#define MPL3115A2_GETOVERSAMPLE(a) ((a >> 3) & 7)
#define MPL3115A2_MAXOVERSAMPLE   7

namespace upm {

/**
 * @brief MPL3115A2 atmospheric pressure sensor library
 * @defgroup mpl3115 libupm-mpl3115
 */

/**
 * @brief C++ API for MPL3115A2 chip (Atmospheric Pressure Sensor)
 *
 * Freescale [MPL3115A2]
 * (http://cache.freescale.com/files/sensors/doc/data_sheet/MPL3115A2.pdf)
 * is a high precision, ultra-low power consumption pressure sensor. It has a
 * range of between 50 and 110 kPa.
 *
 * @ingroup mpl3115a2 i2c
 * @snippet mpl3115a2.cxx Interesting
 * @image html mpl3115a2.jpeg
 */
class MPL3115A2 {
    public:
        /**
         * Instantiates an MPL3115A2 object
         *
         * @param bus number of used bus
         * @param devAddr address of used i2c device
         * @param mode MPL3115A2 oversampling (6 = 64x)
         */
        MPL3115A2(int bus, int devAddr=MPL3115A2_I2C_ADDRESS, uint8_t mode=6);

        /**
         * MPL3115A2 object destructor, basicaly it close i2c connection.
         */
        ~MPL3115A2();

        /**
         * Test the sensor and try to determine if operating by looking
         * for small variations in the value
         */
        int testSensor(void);

        /**
         * Perform a soft RESET of the MPL3115A2 device to ensure
         * it is in a known state.  This function can be used to reset
         * the min/max temperature and pressure values.
         */
        int resetSensor(void);

        /**
         * Dump out the i2c register block to stdout
         */
        void dumpSensor(void);

        /**
         * Initiate a temp/pressure mesasurement and wait for function
         * to complete.  The temp and pressure registers can be read
         * after this call.
         */
        int sampleData(void);

        /**
         * Read a pressure value from the mpl3115a2 [Pa * 100]
         *
         * @param reg base address of pressure register
         * @param pPressure pointer to int32_t buffer for read value
         */
        int getPressureReg (int reg, int32_t* pPressure);

        /**
         * Read a temperature value from the mpl3115a2 [degC * 1000]
         *
         * @param reg base address of temperature register
         * @param pTemperature pointer to int32_t buffer for read value
         */
        int getTempReg (int reg, int32_t* pTemperature);

        /**
         * Read the current pressure value from the mpl3115a2 [Pa * 100]
         * This function should be preceeded by the sampleData() call
         *
         * @param pPressure pointer to int32_t buffer for read value
         */
        int getPressure (int32_t* pPressure);

        /**
         * Read the current temperature value from the mpl3115a2 [degC * 1000]
         * This function should be preceeded by the sampleData() call
         *
         * @param pTemperature pointer to int32_t buffer for read value
         */
        int getTemperature (int32_t* pTemperature);

        /**
         * Read the current temperature and pressure value from the mpl3115a2
         * Function will call the sampleData function
         * [Pa * 100] and [degC * 1000]
         *
         * @param pTemperature pointer to int32_t buffer for read value
         * @param pPressure pointer to int32_t buffer for read value
         */
        int getTempAndPressure (int32_t* pTemperature, int32_t* pPressure);

        /**
         * Read the current pressure and using a known altitude calculate
         * the sea level pressure value [Pa]
         * This function should be preceeded by the sampleData() call
         *
         * @param altitudeMeters Altitude in meters
         */
        float getSealevelPressure(float altitudeMeters = 0.0);

        /**
         * Read the current pressure and using a known sea level pressure
         * calculate the altitude value [m]
         * This function should be preceeded by the sampleData() call
         *
         * @param sealevelPressure Current sea level pressure
         */
        float getAltitude (float sealevelPressure = 101325.0);

        /**
         * Set the oversampling setting (ranges from 0 to 7).  The
         * value represents 2^n samples (ranging from 1 to 128).  The
         * time to calculate the sample is approximately (2^n * 4 + 2) mS
         *
         * @param oversampling New oversampling value
         */
        void setOversampling(uint8_t oversampling);

        /**
         * Returns the current oversampling value
         */
        uint8_t getOversampling(void);

        /**
         * Read the maximum measured temperature [degC * 1000]
         *
         * @param pTemp pointer to int32_t buffer for read value
         */
        int getTemperatureMax(int32_t* pTemp);

        /**
         * Read the minimum measured temperature [degC * 1000]
         *
         * @param pTemp pointer to int32_t buffer for read value
         */
        int getTemperatureMin(int32_t* pTemp);

        /**
         * Read the maximum measured pressure [Pa * 100]
         *
         * @param pTemp pointer to int32_t buffer for read value
         */
        int getPressureMax (int32_t* pPressure);

        /**
         * Read the minimum measured pressure [Pa * 100]
         *
         * @param pTemp pointer to int32_t buffer for read value
         */
        int getPressureMin (int32_t* pPressure);

        /**
         * Convert temperature from degC*1000 to degF*1000
         *
         * @param iTemp Temperature in degC
         */
        int32_t convertTempCtoF(int32_t iTemp);

        /**
         * Convert pressure from Pa*100 to inHg*10000
         * This is set for 15degC (Pa = 0.0002961 in Hg)
         * TODO: Change function to add temp calibration
         *
         * @param iPressure Pressure in Pa
         */
        int32_t convertPaToinHg(int32_t iPressure);

        /**
         * Read two bytes from i2c registers
         *
         * @param reg address of a register
         */
        uint16_t i2cReadReg_16 (int reg);

        /**
         * Write to one byte to i2c register
         *
         * @param reg address of a register
         * @param value byte to be written
         */
        mraa_result_t i2cWriteReg (uint8_t reg, uint8_t value);

        /**
         * Read one byte register
         *
         * @param reg address of a register
         */
        uint8_t i2cReadReg_8 (int reg);

        /**
         * Perform sign extension
         *
         * @param x unsigned int value for extension
         * @param b bit number to use for sign extension
         */
        uint32_t SignExtend32(uint32_t x, uint32_t b);

    private:
        std::string m_name;

        int m_controlAddr;
        int m_bus;
        mraa_i2c_context m_i2ControlCtx;

        uint8_t m_oversampling;
        int32_t m_iPressure;
        int32_t m_iTemperature;
};

}

