#include "drv_brush_motor.h"

#include "main.h"
#include "stm32g4xx_hal.h"
#include "gpio.h"
#include "hrtim.h"
#include "tim.h"

// configurations for brush motor drivers
typedef struct {
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    TIM_HandleTypeDef *encoder_timer;
    uint32_t hrtim_timer_id;
    uint32_t hrtim_timer_index;
    uint32_t hrtim_output;
    uint8_t duty_index;
} brush_motor_drv_cfg_t;

// store current duty for each motor driver
static float motor_duty[BRUSH_MOTOR_DRV_COUNT] = {0.f};

static const brush_motor_drv_cfg_t motor_cfg[BRUSH_MOTOR_DRV_COUNT] = {
    [BRUSH_MOTOR_DRV_LF] = {AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, &htim1,
                            HRTIM_TIMERID_TIMER_A, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, BRUSH_MOTOR_DRV_LF},
    [BRUSH_MOTOR_DRV_RF] = {BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, &htim2,
                            HRTIM_TIMERID_TIMER_B, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, BRUSH_MOTOR_DRV_RF},
    [BRUSH_MOTOR_DRV_RB] = {CIN1_GPIO_Port, CIN1_Pin, CIN2_GPIO_Port, CIN2_Pin, &htim3,
                            HRTIM_TIMERID_TIMER_C, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC1, BRUSH_MOTOR_DRV_RB},
    [BRUSH_MOTOR_DRV_LB] = {DIN1_GPIO_Port, DIN1_Pin, DIN2_GPIO_Port, DIN2_Pin, &htim4,
                            HRTIM_TIMERID_TIMER_D, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1, BRUSH_MOTOR_DRV_LB},
    [BRUSH_MOTOR_DRV_LIFT] = {EIN1_GPIO_Port, EIN1_Pin, EIN2_GPIO_Port, EIN2_Pin, &htim5,
                              HRTIM_TIMERID_TIMER_E, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1, BRUSH_MOTOR_DRV_LIFT},
};

static void hrtim_pwm_set_duty(const brush_motor_drv_cfg_t *cfg, float duty)
{
    HRTIM_Timerx_TypeDef *timer = &hhrtim1.Instance->sTimerxRegs[cfg->hrtim_timer_index];
    uint32_t period = timer->PERxR;

    duty = (duty > 100.f) ? 100.f : (duty < 0.f ? 0.f : duty);
    motor_duty[cfg->duty_index] = duty;

    if (duty <= 0.f) {
        // disable set sorce, enable reset source, force output level to inactive
        CLEAR_BIT(timer->SETx1R, HRTIM_SET1R_PER);
        SET_BIT(timer->RSTx1R, HRTIM_RST1R_CMP1);
        HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, cfg->hrtim_timer_index, cfg->hrtim_output,
                                         HRTIM_OUTPUTLEVEL_INACTIVE);
        return;
    }

    // enable set sorce
    SET_BIT(timer->SETx1R, HRTIM_SET1R_PER);

    if (duty >= 100.f) {
        // disable reset sorce, force output level to active
        CLEAR_BIT(timer->RSTx1R, HRTIM_RST1R_CMP1);
        HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, cfg->hrtim_timer_index, cfg->hrtim_output,
                                         HRTIM_OUTPUTLEVEL_ACTIVE);
        return;
    }

    timer->CMP1xR = (uint32_t)((float)period * duty * 0.01f);
    SET_BIT(timer->RSTx1R, HRTIM_RST1R_CMP1);
}

static void brush_motor_drv_set_duty(const void *ctx, float duty)
{
    hrtim_pwm_set_duty((const brush_motor_drv_cfg_t *)ctx, duty);
}

#ifdef USE_MP6612_DRV

static void brush_motor_drv_set_direction(const void *ctx, int8_t direction)
{
    const brush_motor_drv_cfg_t *cfg = (const brush_motor_drv_cfg_t *)ctx;

    // IN1 == DIR: 1-> forward, 0-> reverse
    // IN2 == nSLEEP: 1-> normal, 0-> sleep, no torque
    if (direction > 0) {
        HAL_GPIO_WritePin(cfg->in1_port, cfg->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, GPIO_PIN_SET);
    } else if (direction < 0) {
        HAL_GPIO_WritePin(cfg->in1_port, cfg->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, GPIO_PIN_RESET);
    }
}

#else

static void brush_motor_drv_set_direction(const void *ctx, int8_t direction)
{
    const brush_motor_drv_cfg_t *cfg = (const brush_motor_drv_cfg_t *)ctx;

    // IN1, IN2: 10: forward, 01: reverse, 11: brake
    if (direction > 0) {
        HAL_GPIO_WritePin(cfg->in1_port, cfg->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, GPIO_PIN_RESET);
    } else if (direction < 0) {
        HAL_GPIO_WritePin(cfg->in1_port, cfg->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(cfg->in1_port, cfg->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, GPIO_PIN_SET);
    }
}

#endif // USE_MP6612_DRV

static uint8_t brush_motor_drv_init(const void *ctx)
{
    const brush_motor_drv_cfg_t *cfg = (const brush_motor_drv_cfg_t *)ctx;

    brush_motor_drv_set_direction(cfg, 0);

    if (HAL_TIM_Encoder_Start(cfg->encoder_timer, TIM_CHANNEL_1 | TIM_CHANNEL_2) != HAL_OK) {
        return 0;
    }

    if (HAL_HRTIM_WaveformCountStart(&hhrtim1, cfg->hrtim_timer_id) != HAL_OK) {
        return 0;
    }
    hhrtim1.Instance->sTimerxRegs[cfg->hrtim_timer_index].CMP1xR = 0;
    if (HAL_HRTIM_WaveformOutputStart(&hhrtim1, cfg->hrtim_output) != HAL_OK) {
        return 0;
    }
    hrtim_pwm_set_duty(cfg, 0.f);

    return 1;
}

static float brush_motor_drv_get_duty(const void *ctx)
{
    const brush_motor_drv_cfg_t *cfg = (const brush_motor_drv_cfg_t *)ctx;

    return motor_duty[cfg->duty_index];
}

/* attention: some timers' CNT register is 16-bit, some is 32-bit, but we only return int16_t here
 * which means the max count we can get is 32767, if the count exceeds this value, it will be wrapped around to negative value
 * So make sure to read encoder count frequently enough to avoid overflow.
 */
static int16_t brush_motor_drv_get_encoder_count(const void *ctx)
{
    const brush_motor_drv_cfg_t *cfg = (const brush_motor_drv_cfg_t *)ctx;
    int16_t count = (int16_t)cfg->encoder_timer->Instance->CNT;

    cfg->encoder_timer->Instance->CNT = 0;
    return count;
}

#define BRUSH_MOTOR_DRIVER(_id) \
    {                           \
        .ctx = &motor_cfg[_id], .init = brush_motor_drv_init, .set_duty = brush_motor_drv_set_duty, .set_direction = brush_motor_drv_set_direction, .get_duty = brush_motor_drv_get_duty, .get_encoder_count = brush_motor_drv_get_encoder_count}

// define export brush motor driver instances
const brush_motor_driver_t LF_motor_driver = BRUSH_MOTOR_DRIVER(BRUSH_MOTOR_DRV_LF);
const brush_motor_driver_t RF_motor_driver = BRUSH_MOTOR_DRIVER(BRUSH_MOTOR_DRV_RF);
const brush_motor_driver_t RB_motor_driver = BRUSH_MOTOR_DRIVER(BRUSH_MOTOR_DRV_RB);
const brush_motor_driver_t LB_motor_driver = BRUSH_MOTOR_DRIVER(BRUSH_MOTOR_DRV_LB);
const brush_motor_driver_t lift_motor_driver = BRUSH_MOTOR_DRIVER(BRUSH_MOTOR_DRV_LIFT);

const brush_motor_driver_t *brush_motor_drv_get(brush_motor_drv_id_e id)
{
    static const brush_motor_driver_t *const drivers[BRUSH_MOTOR_DRV_COUNT] = {
        &LF_motor_driver,
        &RF_motor_driver,
        &RB_motor_driver,
        &LB_motor_driver,
        &lift_motor_driver,
    };

    if ((id < BRUSH_MOTOR_DRV_LF) || (id >= BRUSH_MOTOR_DRV_COUNT)) {
        return 0;
    }

    return drivers[id];
}
