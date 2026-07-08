#ifndef _DRV_BRUSH_MOTOR_H_
#define _DRV_BRUSH_MOTOR_H_

#include "dev_brush_motor.h"

#define USE_MP6612_DRV

typedef enum {
    BRUSH_MOTOR_DRV_LF = 0,
    BRUSH_MOTOR_DRV_RF,
    BRUSH_MOTOR_DRV_RB,
    BRUSH_MOTOR_DRV_LB,
    BRUSH_MOTOR_DRV_LIFT,
    BRUSH_MOTOR_DRV_COUNT,
} brush_motor_drv_id_e;

extern const brush_motor_driver_t LF_motor_driver;
extern const brush_motor_driver_t RF_motor_driver;
extern const brush_motor_driver_t RB_motor_driver;
extern const brush_motor_driver_t LB_motor_driver;
extern const brush_motor_driver_t lift_motor_driver;

const brush_motor_driver_t *brush_motor_drv_get(brush_motor_drv_id_e id);

#endif /* _DRV_BRUSH_MOTOR_H_ */
