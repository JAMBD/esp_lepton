/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Juho Vähä-Herttua (juhovh@iki.fi)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "esp_i2c.h"

#include <esp8266.h>
#include "io.h"
#include "gpio.h"
#include "user_interface.h"

#define DEFAULT_CLOCK     1000  // 100kHz
#define DEFAULT_CS_LIMIT  800     // 800 80MHz cycles

#define HIGH  1
#define LOW   0

#define SCL_GPIO 4
#define SDA_GPIO 5

static uint8 i2c_delay_us;
static uint32 i2c_cslimit;

#define SDA_SET(bit)  if(bit==HIGH){GPIO_DIS_OUTPUT(SDA_GPIO);}else{GPIO_OUTPUT_SET(SDA_GPIO, LOW);}
#define SDA_GET()     GPIO_INPUT_GET(SDA_GPIO)
#define SCL_SET(bit)  if(bit==HIGH){GPIO_DIS_OUTPUT(SCL_GPIO);}else{GPIO_OUTPUT_SET(SCL_GPIO, LOW);}
#define SCL_GET()     GPIO_INPUT_GET(SCL_GPIO)

#define I2C_DELAY()   os_delay_us(i2c_delay_us)
#define I2C_CLOCK_STRETCH() do {                \
  uint32 i=0;                             \
  while (SCL_GET() == 0 && i++ < i2c_cslimit);  \
} while(0)


static uint8 ICACHE_FLASH_ATTR
i2c_write_start()
{
  SCL_SET(HIGH);
  SDA_SET(HIGH);
  if (SDA_GET() == LOW) {
    return 1;
  }
  I2C_DELAY();
  SDA_SET(LOW);
  I2C_DELAY();

  return 0;
}

static void ICACHE_FLASH_ATTR
i2c_write_stop()
{
  SCL_SET(LOW);
  SDA_SET(LOW);
  I2C_DELAY();
  SCL_SET(HIGH);
  I2C_CLOCK_STRETCH();
  I2C_DELAY();
  SDA_SET(HIGH);
  I2C_DELAY();
}

static void ICACHE_FLASH_ATTR
i2c_write_bit(bool bit)
{
  SCL_SET(LOW);
  SDA_SET(bit);
  I2C_DELAY();
  SCL_SET(HIGH);
  I2C_CLOCK_STRETCH();
  I2C_DELAY();
}

static uint8 ICACHE_FLASH_ATTR
i2c_read_bit()
{
  uint8 bit;

  SCL_SET(LOW);
  SDA_SET(HIGH);
  I2C_DELAY();
  SCL_SET(HIGH);
  I2C_CLOCK_STRETCH();
  bit = SDA_GET();
  I2C_DELAY();

  return bit;
}

static uint8 ICACHE_FLASH_ATTR
i2c_write_byte(uint8 byte)
{
  uint8 bit;

  for (bit = 0; bit < 8; bit++) {
    i2c_write_bit(byte & 0x80);
    byte <<= 1;
  }

  // Read either ACK or NACK
  return i2c_read_bit();
}

static uint8 ICACHE_FLASH_ATTR
i2c_read_byte(uint8 nack)
{
  uint8 byte = 0;
  uint8 bit;

  for (bit = 0; bit < 8; bit++) {
    byte = (byte << 1);
    byte |= i2c_read_bit();
  }
  i2c_write_bit(nack);

  return byte;
}

void ICACHE_FLASH_ATTR esp_i2c_flush()
{
  while(true){
    SDA_SET(HIGH);
    if (SDA_GET() == HIGH) {
      return ;
    }
    SCL_SET(LOW);
    I2C_DELAY();
    SCL_SET(HIGH);
  }
}

void ICACHE_FLASH_ATTR
esp_i2c_init()
{
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
  GPIO_DIS_OUTPUT(SCL_GPIO);
  GPIO_DIS_OUTPUT(SDA_GPIO);

  esp_i2c_set_clock(DEFAULT_CLOCK);
  esp_i2c_set_clock_stretch_limit(DEFAULT_CS_LIMIT);
  esp_i2c_flush();
}

void ICACHE_FLASH_ATTR
esp_i2c_stop()
{
}

void ICACHE_FLASH_ATTR
esp_i2c_set_clock(uint32 freq)
{
  // Divide one second (in us) with frequency
  i2c_delay_us = 1000 / (freq / 1000);
}

void ICACHE_FLASH_ATTR
esp_i2c_set_clock_stretch_limit(uint32 limit)
{
  if (system_get_cpu_freq() == SYS_CPU_160MHZ) {
    i2c_cslimit = 2 * limit;
  } else {
    i2c_cslimit = limit;
  }
}


uint8 ICACHE_FLASH_ATTR
esp_i2c_read_buf(uint8 address, uint8 *buf, uint32 len, uint8 send_stop)
{
  uint32 i;

  if (i2c_write_start()) {
    // Received SDA low when writing start
    return 1;
  }
  if (i2c_write_byte(((address << 1) | 1) & 0xFF)) {
    // Received NACK when writing address
    if (send_stop) i2c_write_stop();
    return 2;
  }

  for (i=0; i < len-1; i++) {
    buf[i] = i2c_read_byte(0);
  }
  buf[i] = i2c_read_byte(1);
  if (send_stop) i2c_write_stop();

  return 0;
}

uint8 ICACHE_FLASH_ATTR
esp_i2c_write_buf(uint8 address, uint8 *buf, uint32 len, uint8 send_stop)
{
  uint32 i;

  if (i2c_write_start()) {
    // Received SDA low when writing start
    return 1;
  }
  if (i2c_write_byte((address << 1) & 0xFF)) {
    // Received NACK when writing address
    if (send_stop) i2c_write_stop();
    return 2;
  }

  for (i=0; i < len; i++) {
    if (i2c_write_byte(buf[i])) {
      // Received NACK when writing data
      if (send_stop) i2c_write_stop();
      return 3;
    }
  }
  if (send_stop) i2c_write_stop();

  return 0;
}
