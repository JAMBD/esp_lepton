/**
  ******************************************************************************
  * @file    i2c.c
  * @author  Baoshi <mail(at)ba0sh1(dot)com>
  * @version 0.1
  * @date    Dec 24, 2014
  * @brief   Bit-bang I2C interface for ESP8266
  *
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2015, Baoshi Zhu. All rights reserved.
  * Use of this source code is governed by a BSD-style license that can be
  * found in the LICENSE.txt file.
  *
  * THIS SOFTWARE IS PROVIDED 'AS-IS', WITHOUT ANY EXPRESS OR IMPLIED
  * WARRANTY.  IN NO EVENT WILL THE AUTHOR(S) BE HELD LIABLE FOR ANY DAMAGES
  * ARISING FROM THE USE OF THIS SOFTWARE.
  *
  */

#include <esp8266.h>
#include "driver/i2c.h"


/**
 * @name Pin selection and configuration block
 * @{
 */

//! @brief GPIO MUX for SDA pin
#define SDA_MUX  PERIPHS_IO_MUX_GPIO4_U
//! @brief GPIO FUNC for SDA pin
#define SDA_FUNC FUNC_GPIO4
//! @brief GPIO pin location for SDA pin
#define SDA_PIN  4
//! @brief GPIO bit location for SDA pin
#define SDA_BIT  BIT4

//! @brief GPIO MUX for SCL pin
#define SCL_MUX  PERIPHS_IO_MUX_GPIO5_U
//! @brief GPIO FUNC for SCL pin
#define SCL_FUNC FUNC_GPIO5
//! @brief GPIO pin location for SCL pin
#define SCL_PIN  5
//! @brief GPIO bit location for SCL pin
#define SCL_BIT  BIT5

//! Delay amount in-between bits, with os_delay_us(1) I get ~300kHz I2C clock
#define _DELAY os_delay_us(1)

/** @} */


// this is found in gpio.h from IoT SDK but not in FreeRTOS SDK
#ifndef GPIO_PIN_ADDR
#define GPIO_PIN_ADDR(i) (GPIO_PIN0_ADDRESS + i*4)
#endif

#define _SDA1 GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, SDA_BIT)
#define _SDA0 GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, SDA_BIT)

#define _SCL1 GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, SCL_BIT)
#define _SCL0 GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, SCL_BIT)

#define _SDAX ((GPIO_REG_READ(GPIO_IN_ADDRESS) >> SDA_PIN) & 0x01)
#define _SCLX ((GPIO_REG_READ(GPIO_IN_ADDRESS) >> SCL_PIN) & 0x01)


void ICACHE_FLASH_ATTR i2c_init(void)
{
    // MUX selection
    PIN_FUNC_SELECT(SDA_MUX, SDA_FUNC);
    PIN_FUNC_SELECT(SCL_MUX, SCL_FUNC);

    // Set SDA as OD output
    GPIO_REG_WRITE
    (
        GPIO_PIN_ADDR(GPIO_ID_PIN(SDA_PIN)),
        GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(SDA_PIN))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)
    );
    // Set SCK as OD output
    GPIO_REG_WRITE
    (
        GPIO_PIN_ADDR(GPIO_ID_PIN(SCL_PIN)),
        GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(SCL_PIN))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)
    );
    // Set idle bus high
    _SDA1;
    _SCL1;
    // Set both output
    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | SDA_BIT | SCL_BIT);
    return;
}


bool ICACHE_FLASH_ATTR i2c_start(void)
{
    _SDA1;
    _SCL1;
    _DELAY;
    if (_SDAX == 0) return false; // Bus busy
    _SDA0;
    _DELAY;
    _SCL0;
    return true;
}


void i2c_beginTransmission(uint8_t data){
    i2c_start();
    return i2c_write(data << 1);
}

void i2c_requestFrom(uint8_t data){
    i2c_start();
    return i2c_write(data << 1 | 0x01);
}

void ICACHE_FLASH_ATTR i2c_stop(void)
{
    _SDA0;
    _SCL1;
    _DELAY;
    while (_SCLX == 0); // clock stretching
    _SDA1;
    _DELAY;
}


// return: true - ACK; false - NACK
bool ICACHE_FLASH_ATTR i2c_write(uint8_t data)
{
    uint8_t ibit;
    bool ret;

    for (ibit = 0; ibit < 8; ++ibit)
    {
        if (data & 0x80)
            _SDA1;
        else
            _SDA0;
        _DELAY;
        _SCL1;
        _DELAY;
        data = data << 1;
        _SCL0;
    }
    _SDA1;
    _DELAY;
    _SCL1;
    _DELAY;
    ret = (_SDAX == 0);
    _SCL0;
    _DELAY;

    return ret;
}


uint8_t ICACHE_FLASH_ATTR i2c_read(void)
{
    uint8_t data = 0;
    uint8_t ibit = 8;

    _SDA1;
    while (ibit--)
    {
        data = data << 1;
        _SCL0;
        _DELAY;
        _SCL1;
        _DELAY;
        if (_SDAX)
            data = data | 0x01;
    }
    _SCL0;

    return data;
}


void ICACHE_FLASH_ATTR i2c_set_ack(bool ack)
{
    _SCL0;
    if (ack)
        _SDA0;  // ACK
    else
        _SDA1;  // NACK
    _DELAY;
    // Send clock
    _SCL1;
    _DELAY;
    _SCL0;
    _DELAY;
    // ACK end
    _SDA1;
}

LEP_RESULT DEV_I2C_MasterInit(LEP_UINT16 portID,
        LEP_UINT16 *BaudRate){
    i2c_init();
    return LEP_OK;
}

LEP_RESULT DEV_I2C_MasterClose(){
    return LEP_OK;

}

LEP_RESULT DEV_I2C_MasterReset(void ){
    i2c_stop();
    i2c_init();
    return LEP_OK;

}

LEP_RESULT DEV_I2C_MasterReadData(LEP_UINT16 portID,
        LEP_UINT8   deviceAddress,
        LEP_UINT16  regAddress,            // Lepton Register Address
        LEP_UINT16 *readDataPtr,
        LEP_UINT16  wordsToRead,          // Number of 16-bit words to Read
        LEP_UINT16 *numWordsRead,         // Number of 16-bit words actually Read
        LEP_UINT16 *status
        ){
    return LEP_OK;

}

LEP_RESULT DEV_I2C_MasterWriteData(LEP_UINT16 portID,
        LEP_UINT8   deviceAddress,
        LEP_UINT16  regAddress,            // Lepton Register Address
        LEP_UINT16 *writeDataPtr,
        LEP_UINT16  wordsToWrite,        // Number of 16-bit words to Write
        LEP_UINT16 *numWordsWritten,     // Number of 16-bit words actually written
        LEP_UINT16 *status
        ){
    return LEP_OK;
}

LEP_RESULT DEV_I2C_MasterReadRegister( LEP_UINT16 portID,
        LEP_UINT8  deviceAddress, 
        LEP_UINT16 regAddress,
        LEP_UINT16 *regValue,     // Number of 16-bit words actually written
        LEP_UINT16 *status
        ){
    return LEP_OK;

}

LEP_RESULT DEV_I2C_MasterWriteRegister( LEP_UINT16 portID,
        LEP_UINT8  deviceAddress, 
        LEP_UINT16 regAddress,
        LEP_UINT16 regValue,     // Number of 16-bit words actually written
        LEP_UINT16 *status
        ){
    return LEP_OK;

}

LEP_RESULT DEV_I2C_MasterStatus(void ){
    return LEP_OK;

}
