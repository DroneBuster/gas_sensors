#include "ch.h"
#include "hal.h"

#include "bme280.h"

static bool bme280_init = false;

static uint16_t dig_T1, dig_P1;
static int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7;
static int16_t dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;
static uint8_t dig_H1, dig_H3;
static int8_t dig_H6;

static int32_t t_fine;
static float last_temperature;
static float last_pressure;
static float last_humidity;
static mutex_t bme280_data;

bool read_register(i2caddr_t addr, uint8_t regaddr, uint8_t* data,
        uint8_t rx_len);
bool write_register(i2caddr_t addr, uint8_t regaddr, uint8_t* data,
        uint8_t tx_len);

int32_t BME280_compensate_T_int32(int32_t adc_T);
uint32_t BME280_compensate_P_int64(int32_t adc_P);
uint32_t bme280_compensate_H_int32(int32_t adc_H);

bool init_bme280() {
    if (bme280_init)
        return true;
    uint8_t data = 0xE0;

    //Reset sensor
    write_register(BME280_I2C_ADDRESS, BME280_RESET_REG, &data, 1);
    chThdSleepMilliseconds(100);

    uint8_t chip_id[2];
    if (!read_register(BME280_I2C_ADDRESS, BME280_CHIP_ID_REG, chip_id, 2))
        return false;

    if (chip_id[0] != BME280_DEFAULT_CHIP_ID) //Not BME280
        return false;

    //Read calibration data in bursts

    uint8_t buff[25];
    if (!read_register(BME280_I2C_ADDRESS, BME280_T1_LSB_REG, buff, 25)) //Read from BME280_T1_LSB_REG to BME280_H1_REG (0x88 to 0xA1)
        return false;

    dig_T1 = (buff[1] << 8) | buff[0];
    dig_T2 = (buff[3] << 8) | buff[2];
    dig_T3 = (buff[5] << 8) | buff[4];
    dig_P1 = (buff[7] << 8) | buff[6];
    dig_P2 = (buff[9] << 8) | buff[8];
    dig_P3 = (buff[11] << 8) | buff[10];
    dig_P4 = (buff[13] << 8) | buff[12];
    dig_P5 = (buff[15] << 8) | buff[14];
    dig_P6 = (buff[17] << 8) | buff[16];
    dig_P7 = (buff[19] << 8) | buff[18];
    dig_P8 = (buff[21] << 8) | buff[20];
    dig_P9 = (buff[23] << 8) | buff[22];
    dig_H1 = buff[24];

    if (!read_register(BME280_I2C_ADDRESS, BME280_H2_LSB_REG, buff, 7)) //Read from BME280_H2_LSB_REG to BME280_H6_LSB_REG (0xE1 to 0xE7)
        return false;

    dig_H2 = (buff[1] << 8) | buff[0];
    dig_H3 = buff[2];
    dig_H4 = (buff[3] << 4) | (buff[4] & 0x0F);
    dig_H5 = (buff[5] << 4) | ((buff[4] & 0xF0) >> 4);
    dig_H6 = buff[6];

    //Configure sensor
    data = 0b10000000;
    if (!write_register(BME280_I2C_ADDRESS, BME280_CONFIG_REG, &data, 1))
        return false;
    data = 0b00000001;
    if (!write_register(BME280_I2C_ADDRESS, BME280_CTRL_HUM_REG, &data, 1))
        return false;

    /* Initialize data access mutex */
    chMtxObjectInit(&bme280_data);

    bme280_init = true;
    return true;
}

bool read_register(i2caddr_t addr, uint8_t regaddr, uint8_t* data,
        uint8_t rx_len) {
    msg_t res;
    res = i2cMasterTransmitTimeout(&BME280_BUS, addr, &regaddr, 1, NULL, 0,
            MS2ST(1000));
    if (res != 0)
        return false;

    res = i2cMasterReceiveTimeout(&BME280_BUS, addr, data, rx_len, MS2ST(1000));
    if (res != 0)
        return false;

    return true;
}

bool write_register(i2caddr_t addr, uint8_t regaddr, uint8_t* data,
        uint8_t tx_len) {
    msg_t res;
    uint8_t send_buff[2];
    send_buff[0] = regaddr;
    send_buff[1] = data[0];
    res = i2cMasterTransmitTimeout(&BME280_BUS, addr, send_buff, 2, NULL, 0,
            MS2ST(1000));
    if (res != 0)
        return false;

    return true;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t BME280_compensate_T_int32(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2))
            >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t) dig_T1))
            * ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3))
            >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_compensate_P_int64(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t) t_fine) - 128000;
    var2 = var1 * var1 * (int64_t) dig_P6;
    var2 = var2 + ((var1 * (int64_t) dig_P5) << 17);
    var2 = var2 + (((int64_t) dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t) dig_P3) >> 8)
            + ((var1 * (int64_t) dig_P2) << 12);
    var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) dig_P1) >> 33;
    if (var1 == 0)
        return 0; // avoid exception caused by division by zero
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t) dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t) dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t) dig_P7) << 4);
    return (uint32_t) p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t bme280_compensate_H_int32(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t) 76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t) dig_H4) << 20)
            - (((int32_t) dig_H5) * v_x1_u32r)) + ((int32_t) 16384)) >> 15)
            * (((((((v_x1_u32r * ((int32_t) dig_H6)) >> 10)
                    * (((v_x1_u32r * ((int32_t) dig_H3)) >> 11)
                            + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152))
                    * ((int32_t) dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r
            - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
                    * ((int32_t) dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t) (v_x1_u32r >> 12);
}

bool bme280_measurement(void) {
    if (!bme280_init)
        return false;

    //Start measurement
    uint8_t data = 0b01010101; //Pressure over x 16; Temperature over x 2; Humidity x 1
    write_register(BME280_I2C_ADDRESS, BME280_CTRL_MEAS_REG, &data, 1);

    //Wait for measurement to complete
    uint8_t status_reg[2];
    status_reg[0] = 1;
    chThdSleepMilliseconds(100);
    while (status_reg[0] != 0) {
        read_register(BME280_I2C_ADDRESS, BME280_STATUS_REG, status_reg, 2);
        chThdSleepMilliseconds(5);
    }
    uint8_t buff[8];
    read_register(BME280_I2C_ADDRESS, BME280_PRESS_MSB_REG, buff, 8);

    int32_t adc_P = (buff[0] << 12) | (buff[1] << 4) | ((buff[2] & 0xF0) >> 4);
    int32_t adc_T = (buff[3] << 12) | (buff[4] << 4) | ((buff[5] & 0xF0) >> 4);
    int32_t adc_H = (buff[6] << 8) | buff[7];

    chMtxLock(&bme280_data);
    last_pressure = BME280_compensate_P_int64(adc_P) / 256.0;
    last_temperature = BME280_compensate_T_int32(adc_T) / 100.0;
    last_humidity = bme280_compensate_H_int32(adc_H) / 1024.0;
    chMtxUnlock(&bme280_data);
    return true;
}

float get_baro(void) {
    chMtxLock(&bme280_data);
    float tmp = last_pressure;
    chMtxUnlock(&bme280_data);
    return tmp;
}

float get_tempeture(void) {
    chMtxLock(&bme280_data);
    float tmp = last_temperature;
    chMtxUnlock(&bme280_data);
    return tmp;
}

float get_humidity(void) {
    chMtxLock(&bme280_data);
    float tmp = last_humidity;
    chMtxUnlock(&bme280_data);
    return tmp;
}

