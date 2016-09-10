#ifndef SRC_DRIVERS_ANALOG_H_
#define SRC_DRIVERS_ANALOG_H_

void init_analog(void);
void measure_sensors(void);
void get_analog_sensor_values(float *co, float *no2, float *nh3, float *no, float *so2);

typedef enum {
    CO_SENSOR,
    NO2_SENSOR,
    NH3_SENSOR,
    O3_SENSOR
} sensor_types_t;

#endif /* SRC_DRIVERS_ANALOG_H_ */
