#ifndef SRC_DRIVERS_BME280_H_
#define SRC_DRIVERS_BME280_H_

#define BME280_BUS I2CD1 //Which bus sensor is attached to

#define BME280_I2C_ADDRESS (0x76) //SDO connected to GND
//#define BME280_I2C_ADDRESS (0x77) //SDO connected to VCC

#define BME280_DEFAULT_CHIP_ID (0x60)

//Compensation registers
#define BME280_T1_LSB_REG     (0x88)
#define BME280_T1_MSB_REG     (0x89)
#define BME280_T2_LSB_REG     (0x8A)
#define BME280_T2_MSB_REG     (0x8B)
#define BME280_T3_LSB_REG     (0x8C)
#define BME280_T3_MSB_REG     (0x8D)

#define BME280_P1_LSB_REG     (0x8E)
#define BME280_P1_MSB_REG     (0x8F)
#define BME280_P2_LSB_REG     (0x90)
#define BME280_P2_MSB_REG     (0x91)
#define BME280_P3_LSB_REG     (0x92)
#define BME280_P3_MSB_REG     (0x93)
#define BME280_P4_LSB_REG     (0x94)
#define BME280_P4_MSB_REG     (0x95)
#define BME280_P5_LSB_REG     (0x96)
#define BME280_P5_MSB_REG     (0x97)
#define BME280_P6_LSB_REG     (0x98)
#define BME280_P6_MSB_REG     (0x99)
#define BME280_P7_LSB_REG     (0x9A)
#define BME280_P7_MSB_REG     (0x9B)
#define BME280_P8_LSB_REG     (0x9C)
#define BME280_P8_MSB_REG     (0x9D)
#define BME280_P9_LSB_REG     (0x9E)
#define BME280_P9_MSB_REG     (0x9F)

#define BME280_H1_REG         (0xA1)
#define BME280_H2_LSB_REG     (0xE1)
#define BME280_H2_MSB_REG     (0xE2)
#define BME280_H3_REG         (0xE3)
#define BME280_H4_LSB_REG     (0xE4)
#define BME280_H4_MSB_REG     (0xE5)
#define BME280_H5_LSB_REG     (0xE5)
#define BME280_H5_MSB_REG     (0xE6)
#define BME280_H6_LSB_REG     (0xE7)

//Chip ID register
#define BME280_CHIP_ID_REG    (0xD0)

//Data registers
#define BME280_HUM_LSB_REG    (0xFE)
#define BME280_HUM_MSB_REG    (0xFD)

#define BME280_TEMP_XLSB_REG  (0xFC)
#define BME280_TEMP_LSB_REG   (0xFB)
#define BME280_TEMP_MSB_REG   (0xFA)

#define BME280_PRESS_XLSB_REG (0xF9)
#define BME280_PRESS_LSB_REG  (0xF8)
#define BME280_PRESS_MSB_REG  (0xF7)

//Configuration registers
#define BME280_CONFIG_REG     (0xF5)

//Control registers
#define BME280_CTRL_MEAS_REG  (0xF4)
#define BME280_CTRL_HUM_REG   (0xF2)
#define BME280_RESET_REG      (0xE0)

//Status registers
#define BME280_STATUS_REG     (0xF3)

bool init_bme280(void);
bool bme280_measurement(void); //Run and aquire data from sensor

float get_baro(void); //Get last measurement
float get_tempeture(void);
float get_humidity(void);

#endif /* SRC_DRIVERS_BME280_H_ */
