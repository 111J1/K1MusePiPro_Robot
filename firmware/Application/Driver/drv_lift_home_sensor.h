#ifndef _DRV_LIFT_HOME_SENSOR_H_
#define _DRV_LIFT_HOME_SENSOR_H_

#include <stdint.h>

typedef struct {
    const void *ctx;
    uint8_t (*init)(const void *ctx);
    uint8_t (*get_level)(const void *ctx);
} lift_home_sensor_driver_t;

typedef enum {
    LIFT_HOME_SENSOR_DRV_BOTTOM = 0,
    LIFT_HOME_SENSOR_DRV_MIDDLE,
    LIFT_HOME_SENSOR_DRV_TOP,
    LIFT_HOME_SENSOR_DRV_COUNT,
} lift_home_sensor_drv_id_e;

#define LIFT_HOME_SENSOR_DRV_1 LIFT_HOME_SENSOR_DRV_BOTTOM

extern const lift_home_sensor_driver_t lift_bottom_sensor_driver;
extern const lift_home_sensor_driver_t lift_middle_sensor_driver;
extern const lift_home_sensor_driver_t lift_top_sensor_driver;
extern const lift_home_sensor_driver_t lift_home_sensor_driver;

const lift_home_sensor_driver_t *lift_home_sensor_drv_get(lift_home_sensor_drv_id_e id);

#endif /* _DRV_LIFT_HOME_SENSOR_H_ */
