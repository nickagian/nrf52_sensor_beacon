#ifndef SENSORS_H_
#define SENSORS_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "bme280.h"

int8_t bme280_sensor_init(void *twi_intf);
int8_t bme280_get_measurement(struct bme280_data *comp_data);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* SENSORS_H_ */
/** @}*/