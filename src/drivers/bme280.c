#include "ch.h"
#include "hal.h"

#include "bme280.h"

static bool bme280_init = false;

bool read_register(i2caddr_t addr, uint8_t regaddr, uint8_t* data, uint8_t rx_len);

bool init_bme280()
{
  if(bme280_init)
    return true;

  uint8_t chip_id;
  if(!read_register(BME280_I2C_ADDRESS, BME280_CHIP_ID_REG, &chip_id, 1))
    return false;

  if(chip_id != BME280_DEFAULT_CHIP_ID) //Not BME280
    return false;

  //Read calibration data

  uint8_t buff[25];
  if(!read_register(BME280_I2C_ADDRESS, BME280_T1_LSB_REG, buff, 25)) //Read from BME280_T1_LSB_REG to BME280_H1_REG (0x88 to 0xA1)
    return false;

  if(!read_register(BME280_I2C_ADDRESS, BME280_H2_LSB_REG, buff, 7)) //Read from BME280_H2_LSB_REG to BME280_H6_LSB_REG (0xE1 to 0xE7)
    return false;

  bme280_init = true;
  return true;
}

bool read_register(i2caddr_t addr, uint8_t regaddr, uint8_t* data, uint8_t rx_len)
{
  msg_t res;
  res = i2cMasterTransmitTimeout(&BME280_BUS, addr, &regaddr, 1, NULL, 0, MS2ST(1000));
  if(res != 0)
    return false;

  res = i2cMasterReceiveTimeout(&BME280_BUS, addr, data, rx_len, MS2ST(1000));
  if(res != 0)
    return false;

  return true;
}




