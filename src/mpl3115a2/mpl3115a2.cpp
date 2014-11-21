/*
 * Author: William Penner <william.penner@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 *
 * Based on code adapted from the kernel MPL3115A2 driver and mraa/upm
 * code by: Yevgeniy Kiveisha <yevgeniy.kiveisha@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 *
 * This application code supports the mpl3115a2 digital barometric pressure
 * and temperature sensor from Freescale.  The datasheet is available
 * from their website:
 * http://cache.freescale.com/files/sensors/doc/data_sheet/MPL3115A2.pdf
 *
 * To decrease RMS noise from pressure measurements, the mpl3115 can
 * autonomously calculate the average of up to 128 samples. This is
 * set up by writing to the oversampling sysfs file. Accepted values
 * are 0 to 7 representing 2^n.
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

#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <endian.h>

#include "mpl3115a2.h"

using namespace upm;

MPL3115A2::MPL3115A2 (int bus, int devAddr, uint8_t mode) {
    int id;

    m_name = MPL3115A2_NAME;

    m_controlAddr = devAddr;
    m_bus = bus;

    m_i2ControlCtx = mraa_i2c_init(m_bus);

    mraa_result_t ret = mraa_i2c_address(m_i2ControlCtx, m_controlAddr);
    if (ret != MRAA_SUCCESS) {
        fprintf(stderr, "Error accessing i2c bus\n");
    }

    setOversampling(mode);

    id = i2cReadReg_8(MPL3115A2_WHO_AM_I);
    if (id != MPL3115A2_DEVICE_ID)  {
        fprintf(stdout, "Incorrect device id - read: 0x%02x\n", id);
    }
}

MPL3115A2::~MPL3115A2() {
    mraa_i2c_stop(m_i2ControlCtx);
}

/*
 * Function to test the device and verify that is appears operational
 * Typically functioning sensors will return "noisy" values and would
 * be expected to change a bit.  This fuction will check for this
 * variation.
 */

int
MPL3115A2::testSensor(void)
{
    int i, iTries;
    int iError = 0;
    int32_t iVal, pressure, temperature;
    int32_t iPMin, iPMax, iTMin, iTMax;

    fprintf(stdout, "Executing Sensor Test.\n" );

    sampleData();
    getPressure(&pressure);
    getTemperature(&temperature);
    iPMin = iPMax = pressure;
    iTMin = iTMax = temperature;

    iTries = 20;
    do {
        sampleData();
        getPressure(&pressure);
        getTemperature(&temperature);
        if (pressure < iPMin)    iPMin = pressure;
        if (pressure > iPMax)    iPMax = pressure;
        if (temperature < iTMin) iTMin = temperature;
        if (temperature > iTMax) iTMax = temperature;
    }
    while(iPMin == iPMax && iTMin == iTMax && --iTries);

    if (iPMin == iPMax && iTMin == iTMax) {
        fprintf(stdout, "  Warning - sensor values not changing.\n" );
        return -1;
    }

    fprintf(stdout, "  Test complete.\n");

    return 0;
}

/*
 * Function to dump out the i2c register block to the screen
 */

void
MPL3115A2::dumpSensor(void)
{
    int i, j, ival;

    fprintf(stdout, "Dumping i2c block from %s\n", MPL3115A2_NAME);
    for (i=0; i < 256; i+=16) {
        fprintf(stdout, "  %02x: ", i);
        for (j=i; j < i+16; j++) {
            fprintf(stdout, "%02x ", i2cReadReg_8(j));
        }
        fprintf(stdout, "\n");
    }
}

/*
 * Function used to soft RESET the MPL3115A2 device to ensure
 * it is in a known state.  This function can be used to reset
 * the min/max temperature and pressure values.
 */

int
MPL3115A2::resetSensor(void)
{
    fprintf(stdout, "Resetting MPL3115A2 device\n" );
    i2cWriteReg(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_RESET);
    usleep(50000);
    i2cWriteReg(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_RESET |
            MPL3115A2_SETOVERSAMPLE(m_oversampling));

    return 0;
}

/*
 * Function will initiate a measurement cycle and will delay until
 * until the data is ready.
 */

int
MPL3115A2::sampleData(void)
{
    int val;
    mraa_result_t ret;
    int tries = 15;
    uint32_t us_delay;

    // trigger measurement
    ret = i2cWriteReg(MPL3115A2_CTRL_REG1,
            MPL3115A2_CTRL_OST | MPL3115A2_SETOVERSAMPLE(m_oversampling));
    if (MRAA_SUCCESS != ret) {
        fprintf(stdout, "Write to trigger measurement failed\n");
        return -1;
    }

    // Calculate and delay the appopriate time for the measurement
    us_delay = ((1 << m_oversampling) * 4 + 2) * 1000;
    usleep(us_delay);

    // Loop waiting for the ready bit to become active
    while (tries-- > 0) {
        val = i2cReadReg_8(MPL3115A2_CTRL_REG1);
        if (val < 0) {
            fprintf(stdout,"Error reading CTRL_REG1\n");
            return -1;
        }

        /* wait for data ready, i.e. OST cleared */
        if (!(val & MPL3115A2_CTRL_OST))
            break;
        usleep(20000);
    }
    if (tries < 0) {
        std::cout << "Device timeout during measurement" << std::endl;
        return -1;
    }

    return 0;
}

/*
 * The following functions will return the various forms of data from
 * the i2c device and will format appropriately.  The raw data functions
 * will contain either the register value or a negative value on error.
 */

int
MPL3115A2::getPressureReg(int reg, int32_t* pPressure) {
    int32_t itemp;

    itemp  = i2cReadReg_16(reg);
    if (itemp < 0) {
        fprintf(stdout, "Error reading pressure register\n");
        return -1;
    }
    itemp = (itemp << 8) | (uint32_t)i2cReadReg_8(reg+2);

    if (pPressure != NULL)
        *pPressure = itemp * 100 / 64;

    return 0;
}

/*
 * Function return is the mpaa_result_t error status
 */

int
MPL3115A2::getTempReg(int reg, int32_t* pTemperature) {
    int32_t itemp;

    itemp = i2cReadReg_16(reg);
    if (itemp < 0) {
        fprintf(stdout, "Error reading pressure register\n");
        return -1;
    }
    if (pTemperature != NULL) {
        itemp = SignExtend32(itemp, 15);
        *pTemperature = itemp * 1000 / 256;
    }

    return 0;
}

/*
 * Return: return status (non-zero on error)
 * pPressure: Pressure [Pa] * 100
 */

int
MPL3115A2::getPressure (int32_t* pPressure) {
    int ret;

    // Trigger request to make a measurement
    ret = sampleData();
    if (ret < 0) {
        fprintf(stdout, "Error reading pressure\n");
        return -1;
    }

    ret = getPressureReg(MPL3115A2_OUT_PRESS, &m_iPressure);
    if (ret < 0)
        return -1;

    if (pPressure != NULL)
        *pPressure = m_iPressure;

    return 0;
}

/*
 * Return: return status (non-zero on error)
 * pTemperature: Temperature [degC] * 1000
 */

int
MPL3115A2::getTemperature(int32_t* pTemperature) {
    int ret;

    // Trigger request to make a measurement
    ret = sampleData();
    if (ret < 0) {
        fprintf(stdout, "Error reading temperature\n");
        return -1;
    }

    ret = getTempReg(MPL3115A2_OUT_TEMP, &m_iTemperature);
    if (ret < 0)
        return -1;

    if (pTemperature != NULL)
        *pTemperature = m_iTemperature;

    return 0;
}

/*
 * Return: return status (non-zero on error)
 * pTemperature: Temperature [degC] * 1000
 * pPressure:    Pressure [Pa] * 100
 */

int
MPL3115A2::getTempAndPressure (int32_t* pTemperature, int32_t* pPressure) {
    int ret;

    // Trigger request to make a measurement
    ret = sampleData();
    if (ret < 0) {
        fprintf(stdout, "Error reading temperature\n");
        return -1;
    }

    ret = getPressureReg(MPL3115A2_OUT_PRESS, &m_iPressure);
    if (ret < 0)
        return -1;

    ret = getTempReg(MPL3115A2_OUT_TEMP, &m_iTemperature);
    if (ret < 0)
        return -1;

    if (pPressure != NULL)
        *pPressure = m_iPressure;

    if (pTemperature != NULL)
        *pTemperature = m_iTemperature;

    return 0;
}

float
MPL3115A2::getSealevelPressure(float altitudeMeters) {
    float fPressure = (float)m_iPressure / 100.0;
    return fPressure / pow(1.0-altitudeMeters/44330, 5.255);
}

float
MPL3115A2::getAltitude (float sealevelPressure) {
    float fPressure = (float)m_iPressure / 100.0;
    return 44330 * (1.0 - pow(fPressure /sealevelPressure,0.1903));
}

/*
 * This function sets the chip-internal oversampling. Valid values are 0..7
 * The chip will use 2^oversampling samples for internal averaging.
 * This influences the measurement time and the accuracy; larger values
 * increase both. The datasheet gives an overview on how measurement time,
 * accuracy and noise correlate.
 */

void
MPL3115A2::setOversampling(uint8_t oversampling)
{
    int ret;

    if (oversampling > MPL3115A2_MAXOVERSAMPLE)
        oversampling = MPL3115A2_MAXOVERSAMPLE;

    m_oversampling = oversampling;

    return;
}

/*
 * Returns the currently selected oversampling. Range: 0..7
 */

uint8_t
MPL3115A2::getOversampling(void)
{
    return m_oversampling;
}

int
MPL3115A2::getTemperatureMax(int32_t* pTemperature)
{
    int ret;
    int32_t temp;

    ret = getTempReg(MPL3115A2_T_MAX, &temp);
    if (ret < 0) {
        fprintf(stdout, "Error reading max temp\n");
        return -1;
    }

    if (pTemperature != NULL)
        *pTemperature = temp;

    return 0;
}

int
MPL3115A2::getTemperatureMin(int32_t* pTemperature)
{
    int ret;
    int32_t temp;

    ret = getTempReg(MPL3115A2_T_MIN, &temp);
    if (ret < 0) {
        fprintf(stdout, "Error reading min temp\n");
        return -1;
    }

    if (pTemperature != NULL)
        *pTemperature = temp;

    return 0;
}

int
MPL3115A2::getPressureMax(int32_t* pPressure)
{
    int ret;
    int32_t temp;

    ret = getPressureReg(MPL3115A2_P_MAX, &temp);
    if (ret < 0) {
        fprintf(stdout, "Error reading max pressure\n");
        return -1;
    }

    if (pPressure != NULL)
        *pPressure = temp;

    return 0;
}

int
MPL3115A2::getPressureMin(int32_t* pPressure)
{
    int ret;
    int32_t temp;

    ret = getPressureReg(MPL3115A2_P_MIN, &temp);
    if (ret < 0) {
        fprintf(stdout, "Error reading min pressure\n");
        return -1;
    }

    if (pPressure != NULL)
        *pPressure = temp;

    return 0;
}

int32_t
MPL3115A2::convertTempCtoF(int32_t iTemp)
{
    return(iTemp * 9 / 5 + 32000);
}

/*
 * This is set for 15degC (Pa = 0.0002961 in Hg)
 */
int32_t
MPL3115A2::convertPaToinHg(int32_t iPressure)
{
    return(iPressure * 10000 / 337724);
}

/*
 * Function used to perform sign extension
 */

uint32_t
MPL3115A2::SignExtend32(uint32_t x, uint32_t b)
{
    return x & (1u << (b - 1)) ? x | ~((1u << b) - 1) :
            x & ((1u << b) - 1);
}

/*
 * Functions to read and write data to the i2c device
 */

mraa_result_t
MPL3115A2::i2cWriteReg (uint8_t reg, uint8_t value) {
    mraa_result_t error = MRAA_SUCCESS;

    uint8_t data[2] = { reg, value };
    mraa_i2c_address (m_i2ControlCtx, m_controlAddr);
    error = mraa_i2c_write (m_i2ControlCtx, data, 2);

    return error;
}

/*
 * Function to read 16 bits starting at reg.  This function
 * was replaced due to functionality of using read() to
 * access i2c data.
 */
uint16_t
MPL3115A2::i2cReadReg_16 (int reg) {
    uint16_t data;

    mraa_i2c_address(m_i2ControlCtx, m_controlAddr);
    data  = (uint16_t)mraa_i2c_read_byte_data(m_i2ControlCtx, reg) << 8;
    data |= (uint16_t)mraa_i2c_read_byte_data(m_i2ControlCtx, reg+1);

    return data;
}

/*
 * Function to read 8 bits starting at reg.  This function
 * was replaced due to functionality of using read() to
 * access i2c data.
 */
uint8_t
MPL3115A2::i2cReadReg_8 (int reg) {
    mraa_i2c_address(m_i2ControlCtx, m_controlAddr);
    return mraa_i2c_read_byte_data(m_i2ControlCtx, reg);
}
