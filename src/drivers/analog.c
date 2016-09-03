#include "ch.h"
#include "hal.h"

#include "analog.h"

#define ADC_GRP1_NUM_CHANNELS   3
#define ADC_GRP1_BUF_DEPTH      8

#define REFERENCE_VOLTAGE 3.3f
#define SWITCH_TRESHOLD 2.0f
#define ADC_BITS 4096

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

float measureResistance(void);
float get_voltage(adcsample_t sample);
void select_sensor(sensor_types_t sensor);

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    IN0.
 */
static const ADCConversionGroup adcResOnly = {
FALSE,
1,
NULL,
adcerrorcallback,
0, 0, /* CR1, CR2 */
0, /* SMPR1 */
ADC_SMPR2_SMP_AN0(ADC_SAMPLE_41P5), /* SMPR2 */
ADC_SQR1_NUM_CH(1), /* SQR1 */
0, /* SQR2 */
ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)  /* SQR3 */
};

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 3 channel, SW triggered.
 * Channels:    IN1, IN2.
 */
static const ADCConversionGroup adcElChemOnlu = {
FALSE,
2,
NULL,
adcerrorcallback,
0, 0, /* CR1, CR2 */
0, /* SMPR1 */
ADC_SMPR2_SMP_AN1(ADC_SAMPLE_41P5) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_41P5), /* SMPR2 */
ADC_SQR1_NUM_CH(1), /* SQR1 */
0, /* SQR2 */
ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN2)  /* SQR3 */
};

void measure_sensors(void) {
    uint16_t resistance_avg = 0;
    uint16_t no_gas_avg = 0;
    uint16_t so2_gas_avg = 0;

    float co_gas;
    float no2_gas;
    float nh3_gas;

    select_sensor(CO_SENSOR);
    co_gas = measureResistance();

    select_sensor(NO2_SENSOR);
    no2_gas = measureResistance();

    select_sensor(NH3_SENSOR);
    nh3_gas = measureResistance();



    adcConvert(&ADCD1, &adcElChemOnlu, samples1, ADC_GRP1_BUF_DEPTH);
    for (int i = 0; i < 2 * ADC_GRP1_BUF_DEPTH; i += 2) {
        no_gas_avg += samples1[i];
    }
    no_gas_avg /= ADC_GRP1_BUF_DEPTH;
    for (int i = 1; i < 2 * ADC_GRP1_BUF_DEPTH; i += 2) {
        so2_gas_avg += samples1[i];
    }
    so2_gas_avg /= ADC_GRP1_BUF_DEPTH;

}

void select_sensor(sensor_types_t sensor) {
    switch(sensor) {
        case CO_SENSOR: {
            palClearPad(GPIOB, GPIOB_SEN_SEL1);
            palClearPad(GPIOB, GPIOB_SEN_SEL2);
            palClearPad(GPIOB, GPIOB_SEN_SEL3);
            break;
        }
        case NO2_SENSOR: {
            palSetPad(GPIOB, GPIOB_SEN_SEL1);
            palClearPad(GPIOB, GPIOB_SEN_SEL2);
            palClearPad(GPIOB, GPIOB_SEN_SEL3);
            break;
        }
        case NH3_SENSOR: {
            palClearPad(GPIOB, GPIOB_SEN_SEL1);
            palSetPad(GPIOB, GPIOB_SEN_SEL2);
            palClearPad(GPIOB, GPIOB_SEN_SEL3);
            break;
        }
        case O3_SENSOR: {
            palSetPad(GPIOB, GPIOB_SEN_SEL1);
            palSetPad(GPIOB, GPIOB_SEN_SEL2);
            palClearPad(GPIOB, GPIOB_SEN_SEL3);
            break;

        }
    }
}

float measure_resistance_voltage(void) {
    adcConvert(&ADCD1, &adcResOnly, samples1, ADC_GRP1_BUF_DEPTH);
    uint32_t resistance_avg = 0;

    for (int i = 0; i < ADC_GRP1_BUF_DEPTH; i++) {
        resistance_avg += samples1[i];
    }
    resistance_avg /= ADC_GRP1_BUF_DEPTH;
    return get_voltage((adcsample_t)resistance_avg);
}

float measureResistance(void) {
    palSetPadMode(GPIOB, GPIOB_1M_MES_SEL, PAL_MODE_INPUT);
    palSetPadMode(GPIOB, GPIOB_100K_MES_SEL, PAL_MODE_INPUT);
    palSetPadMode(GPIOB, GPIOB_6K8_MES_SEL, PAL_MODE_INPUT);
    palClearPad(GPIOB, GPIOB_1M_MES_SEL);
    palClearPad(GPIOB, GPIOB_100K_MES_SEL);
    palClearPad(GPIOB, GPIOB_6K8_MES_SEL);

    float resistance = 0.0f; //In ohms

    float res_voltage = measure_resistance_voltage(); //Using all resistors

    if(res_voltage < SWITCH_TRESHOLD) {
        /* Switch to 1M resistor */
        palSetPadMode(GPIOB, GPIOB_1M_MES_SEL, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPad(GPIOB,GPIOB_1M_MES_SEL);
        res_voltage = measure_resistance_voltage(); //Using 1M+100K+6K8
        if(res_voltage < SWITCH_TRESHOLD) {
            /* Switch to 100K resistor */
           palSetPadMode(GPIOB, GPIOB_100K_MES_SEL, PAL_MODE_OUTPUT_PUSHPULL);
           palSetPad(GPIOB,GPIOB_100K_MES_SEL);
           res_voltage = measure_resistance_voltage(); //Using 100K+6K8
           if(res_voltage < SWITCH_TRESHOLD) {
                /* Switch to 6K8 resistor */
                palSetPadMode(GPIOB, GPIOB_6K8_MES_SEL, PAL_MODE_OUTPUT_PUSHPULL);
                palSetPad(GPIOB, GPIOB_6K8_MES_SEL);
                res_voltage = measure_resistance_voltage(); //Using 6K8
                /* calculate resistance with  6800 ohm */
                resistance = 6800.0f * (1/((3.3f/res_voltage) - 1));
           } else {
               /* calculate resistance with  106800 ohm */
               resistance = 106800.0f * (1/((3.3f/res_voltage) - 1));
           }

        } else {
            /* calculate resistance with  1306800 ohm */
            resistance = 1306800.0f * (1/((3.3f/res_voltage) - 1));
        }
    } else {
        /* calculate resistance with  11306800 ohm */
        resistance = 11306800.0f * (1/((3.3f/res_voltage) - 1));
    }

    return resistance;
}

float get_voltage(adcsample_t sample)
{
    return (REFERENCE_VOLTAGE / ADC_BITS) * sample;
}



void init_analog(void) {
    adcStart(&ADCD1, NULL);
    return;
}



