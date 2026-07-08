#include "task_oled.h"

#include "cmsis_os.h"
#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include "mdl_chassis.h"
#include "task_arm.h"
#include "task_chassis.h"
#include "task_command.h"
#include "task_lift.h"
#include "task_peripheral.h"

#define OLED_TASK_PERIOD_MS 100U
#define OLED_PAGE_COUNT 4U
#define OLED_LINE_HEIGHT 10U
#define OLED_LINE_BUF_SIZE 24U
#define OLED_VALUE_BUF_SIZE 12U
#define OLED_KEY_DEBOUNCE_COUNT 1U
#define OLED_UART_ONLINE_TIMEOUT_MS 1000U
#define OLED_UART1_TIMEOUT_LIMIT 5UL

extern chassis_t chassis;
const chassis_t *chassis_ptr = &chassis;

extern volatile lift_state_snapshot_t g_lift_state;
const volatile lift_state_snapshot_t *lift_state_ptr = &g_lift_state;

extern volatile arm_state_snapshot_t g_arm_state;
const volatile arm_state_snapshot_t *arm_state_ptr = &g_arm_state;

extern volatile peripheral_state_snapshot_t g_peripheral_state;
const volatile peripheral_state_snapshot_t *peripheral_state_ptr = &g_peripheral_state;

extern volatile command_uart_state_t g_command_uart_state;
const volatile command_uart_state_t *command_uart_state_ptr = &g_command_uart_state;

static char oled_line[OLED_LINE_BUF_SIZE];
static char oled_value0[OLED_VALUE_BUF_SIZE];
static char oled_value1[OLED_VALUE_BUF_SIZE];
static uint8_t oled_line_index;
static uint8_t oled_key_sample_pressed;
static uint8_t oled_key_stable_pressed;
static uint8_t oled_key_debounce_count;

static int32_t oled_float_to_scaled(float value, uint16_t scale)
{
    float scaled = value * (float)scale;

    return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) : (int32_t)(scaled - 0.5f);
}

static void oled_append_char(char *buf, uint8_t buf_size, uint8_t *index, char ch)
{
    if ((buf == 0) || (index == 0) || (buf_size == 0U)) {
        return;
    }
    if (*index >= (uint8_t)(buf_size - 1U)) {
        return;
    }

    buf[*index] = ch;
    (*index)++;
    buf[*index] = '\0';
}

static void oled_append_str_to_buf(char *buf, uint8_t buf_size, uint8_t *index, const char *str)
{
    if (str == 0) {
        return;
    }

    while (*str != '\0') {
        oled_append_char(buf, buf_size, index, *str);
        str++;
    }
}

static void oled_append_uint_to_buf(char *buf, uint8_t buf_size, uint8_t *index,
                                    uint32_t value, uint8_t min_width)
{
    char digit[10];
    uint8_t digit_count = 0U;

    do {
        digit[digit_count] = (char)('0' + (value % 10U));
        value /= 10U;
        digit_count++;
    } while ((value != 0U) && (digit_count < sizeof(digit)));

    while (digit_count < min_width) {
        digit[digit_count] = '0';
        digit_count++;
    }

    while (digit_count > 0U) {
        digit_count--;
        oled_append_char(buf, buf_size, index, digit[digit_count]);
    }
}

static void oled_format_value(char *buf, uint8_t buf_size, float value, uint8_t frac_digits)
{
    uint16_t scale = (frac_digits == 3U) ? 1000U : 100U;
    int32_t scaled = oled_float_to_scaled(value, scale);
    uint8_t negative = 0U;
    uint32_t abs_scaled;
    uint32_t integer;
    uint32_t fraction;
    uint8_t index = 0U;

    if ((buf == 0) || (buf_size == 0U)) {
        return;
    }
    buf[0] = '\0';

    if (scaled < 0) {
        negative = 1U;
        abs_scaled = (uint32_t)(-scaled);
    } else {
        abs_scaled = (uint32_t)scaled;
    }

    integer = abs_scaled / scale;
    fraction = abs_scaled % scale;

    if (negative != 0U) {
        oled_append_char(buf, buf_size, &index, '-');
    }
    oled_append_uint_to_buf(buf, buf_size, &index, integer, 1U);
    oled_append_char(buf, buf_size, &index, '.');
    if (frac_digits == 3U) {
        oled_append_uint_to_buf(buf, buf_size, &index, fraction, 3U);
    } else {
        oled_append_uint_to_buf(buf, buf_size, &index, fraction, 2U);
    }
}

static void oled_format_signed_centi(char *buf, uint8_t buf_size, int16_t value)
{
    int32_t scaled = value;
    uint32_t abs_scaled;
    uint8_t index = 0U;

    if ((buf == 0) || (buf_size == 0U)) {
        return;
    }
    buf[0] = '\0';

    if (scaled < 0) {
        oled_append_char(buf, buf_size, &index, '-');
        abs_scaled = (uint32_t)(-scaled);
    } else {
        abs_scaled = (uint32_t)scaled;
    }

    oled_append_uint_to_buf(buf, buf_size, &index, abs_scaled / 100UL, 1U);
    oled_append_char(buf, buf_size, &index, '.');
    oled_append_uint_to_buf(buf, buf_size, &index, abs_scaled % 100UL, 2U);
}

static void oled_format_unsigned_centi(char *buf, uint8_t buf_size, uint16_t value)
{
    uint8_t index = 0U;

    if ((buf == 0) || (buf_size == 0U)) {
        return;
    }
    buf[0] = '\0';

    oled_append_uint_to_buf(buf, buf_size, &index, (uint32_t)value / 100UL, 1U);
    oled_append_char(buf, buf_size, &index, '.');
    oled_append_uint_to_buf(buf, buf_size, &index, (uint32_t)value % 100UL, 2U);
}

static const char *oled_move_cs_string(chassis_move_CS_e move_cs)
{
    return (move_cs == CHASSIS_MOVE_MODE_WCS) ? "WCS" : "LCS";
}

static const char *oled_power_state_string(peripheral_power_state_e state)
{
    switch (state) {
    case PERIPHERAL_POWER_NORMAL:
        return "OK";
    case PERIPHERAL_POWER_LOW:
        return "LOW";
    case PERIPHERAL_POWER_CRITICAL:
        return "CRIT";
    case PERIPHERAL_POWER_FAULT:
        return "FAULT";
    default:
        return "UNK";
    }
}

static void oled_write_line(uint8_t line_index, const char *line)
{
    ssd1306_SetCursor(0U, (uint8_t)(line_index * OLED_LINE_HEIGHT));
    (void)ssd1306_WriteString((char *)line, Font_7x10, White);
}

static void oled_line_begin(const char *str)
{
    oled_line_index = 0U;
    oled_line[0] = '\0';
    oled_append_str_to_buf(oled_line, sizeof(oled_line), &oled_line_index, str);
}

static void oled_line_append_str(const char *str)
{
    oled_append_str_to_buf(oled_line, sizeof(oled_line), &oled_line_index, str);
}

static void oled_line_append_uint(uint32_t value)
{
    oled_append_uint_to_buf(oled_line, sizeof(oled_line), &oled_line_index, value, 1U);
}

static const char *oled_ok_err_string(uint8_t has_fault)
{
    return (has_fault == 0U) ? "OK" : "ERR";
}

static const char *oled_online_string(uint8_t online)
{
    return (online == 0U) ? "LOST" : "OK";
}

static const char *oled_gas_string(uint8_t gas_detected)
{
    return (gas_detected == 0U) ? "OK" : "ALARM";
}

static const char *oled_sht31_status_string(peripheral_sht31_status_e status)
{
    switch (status) {
    case PERIPHERAL_SHT31_STATUS_OK:
        return "OK";
    case PERIPHERAL_SHT31_STATUS_WAIT:
        return "WAIT";
    case PERIPHERAL_SHT31_STATUS_ERR:
    default:
        return "ERR";
    }
}

static uint8_t oled_tick_recent(uint32_t now_tick, uint32_t last_tick, uint32_t timeout_ms)
{
    if (last_tick == 0UL) {
        return 0U;
    }

    return ((uint32_t)(now_tick - last_tick) <= timeout_ms) ? 1U : 0U;
}

static uint8_t oled_uart1_is_online(void)
{
    if (arm_state_ptr->runtime.tick_ms == 0UL) {
        return 0U;
    }
    if (arm_state_ptr->detail.consecutive_timeout_count >= OLED_UART1_TIMEOUT_LIMIT) {
        return 0U;
    }
    if (arm_state_ptr->detail.bus_last_status == STS_SERVO_ERR_TIMEOUT) {
        return 0U;
    }

    return 1U;
}

static uint8_t oled_key_next_page_event(void)
{
    uint8_t pressed = (HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if (pressed == oled_key_sample_pressed) {
        if (oled_key_debounce_count < OLED_KEY_DEBOUNCE_COUNT) {
            oled_key_debounce_count++;
        }
    } else {
        oled_key_sample_pressed = pressed;
        oled_key_debounce_count = 0U;
    }

    if ((oled_key_debounce_count >= OLED_KEY_DEBOUNCE_COUNT) &&
        (oled_key_stable_pressed != oled_key_sample_pressed)) {
        oled_key_stable_pressed = oled_key_sample_pressed;
        if (oled_key_stable_pressed != 0U) {
            return 1U;
        }
    }

    return 0U;
}

static void oled_render_summary(void)
{
    uint32_t now_tick = osKernelGetTickCount();
    uint8_t uart1_online = oled_uart1_is_online();
    uint8_t uart2_online = oled_tick_recent(now_tick,
                                            command_uart_state_ptr->host_last_valid_tick,
                                            OLED_UART_ONLINE_TIMEOUT_MS);
    uint8_t uart3_online = oled_tick_recent(now_tick,
                                            command_uart_state_ptr->bt_last_valid_tick,
                                            OLED_UART_ONLINE_TIMEOUT_MS);

    oled_write_line(0U, "P1 SYS");

    oled_line_begin("PWR:");
    oled_line_append_uint(peripheral_state_ptr->runtime.power_mv);
    oled_line_append_str(" ");
    oled_line_append_str(oled_power_state_string(peripheral_state_ptr->runtime.power_state));
    oled_write_line(1U, oled_line);

    oled_line_begin("U1:");
    oled_line_append_str(oled_online_string(uart1_online));
    oled_line_append_str(" U2:");
    oled_line_append_str(oled_online_string(uart2_online));
    oled_write_line(2U, oled_line);

    oled_line_begin("U3:");
    oled_line_append_str(oled_online_string(uart3_online));
    oled_write_line(3U, oled_line);

    oled_line_begin("ARM:");
    oled_line_append_str(oled_ok_err_string(arm_state_ptr->runtime.has_fault));
    oled_line_append_str(" LIFT:");
    oled_line_append_str(oled_ok_err_string(lift_state_ptr->runtime.has_fault));
    oled_write_line(4U, oled_line);

    oled_line_begin("GAS:");
    oled_line_append_str(oled_gas_string(peripheral_state_ptr->runtime.gas_detected));
    oled_write_line(5U, oled_line);
}

static void oled_render_motion(void)
{
    oled_write_line(0U, "P2 MOTION");

    oled_format_value(oled_value0, sizeof(oled_value0), chassis_ptr->current_V, 2U);
    oled_line_begin("CHS Spd:");
    oled_line_append_str(oled_value0);
    oled_write_line(1U, oled_line);

    oled_format_value(oled_value0, sizeof(oled_value0), chassis_ptr->current_omega, 2U);
    oled_line_begin("Rot:");
    oled_line_append_str(oled_value0);
    oled_line_append_str(" ");
    oled_line_append_str(oled_move_cs_string(chassis_ptr->move_CS));
    oled_write_line(2U, oled_line);

    oled_format_value(oled_value0, sizeof(oled_value0), lift_state_ptr->runtime.current_z, 3U);
    oled_line_begin("Lift:");
    oled_line_append_str(oled_value0);
    oled_write_line(3U, oled_line);

    oled_format_value(oled_value0, sizeof(oled_value0), lift_state_ptr->runtime.target_z, 3U);
    oled_line_begin("Tgt:");
    oled_line_append_str(oled_value0);
    oled_write_line(4U, oled_line);

    oled_line_begin("Home:");
    oled_line_append_uint(lift_state_ptr->runtime.is_homed);
    oled_line_append_str(" Fault:");
    oled_line_append_uint(lift_state_ptr->runtime.has_fault);
    oled_write_line(5U, oled_line);
}

static void oled_render_arm(void)
{
    oled_write_line(0U, "P3 ARM");

    oled_format_value(oled_value0, sizeof(oled_value0), arm_state_ptr->runtime.current_xyz[0], 3U);
    oled_format_value(oled_value1, sizeof(oled_value1), arm_state_ptr->runtime.target_xyz[0], 3U);
    oled_line_begin("X:");
    oled_line_append_str(oled_value0);
    oled_line_append_str("/");
    oled_line_append_str(oled_value1);
    oled_write_line(1U, oled_line);

    oled_format_value(oled_value0, sizeof(oled_value0), arm_state_ptr->runtime.current_xyz[1], 3U);
    oled_format_value(oled_value1, sizeof(oled_value1), arm_state_ptr->runtime.target_xyz[1], 3U);
    oled_line_begin("Y:");
    oled_line_append_str(oled_value0);
    oled_line_append_str("/");
    oled_line_append_str(oled_value1);
    oled_write_line(2U, oled_line);

    oled_format_value(oled_value0, sizeof(oled_value0), arm_state_ptr->runtime.current_xyz[2], 3U);
    oled_format_value(oled_value1, sizeof(oled_value1), arm_state_ptr->runtime.target_xyz[2], 3U);
    oled_line_begin("Z:");
    oled_line_append_str(oled_value0);
    oled_line_append_str("/");
    oled_line_append_str(oled_value1);
    oled_write_line(3U, oled_line);

    oled_format_value(oled_value0, sizeof(oled_value0), arm_state_ptr->runtime.gripper_rad, 2U);
    oled_line_begin("G:");
    oled_line_append_str(oled_value0);
    oled_write_line(4U, oled_line);

    oled_line_begin("ARM:");
    oled_line_append_str(oled_ok_err_string(arm_state_ptr->runtime.has_fault));
    oled_write_line(5U, oled_line);
}

static void oled_render_peripheral(void)
{
    oled_write_line(0U, "P4 PERIPH");

    oled_line_begin("PWR:");
    oled_line_append_uint(peripheral_state_ptr->runtime.power_mv);
    oled_line_append_str(" ");
    oled_line_append_str(oled_power_state_string(peripheral_state_ptr->runtime.power_state));
    oled_write_line(1U, oled_line);

    oled_line_begin("GAS:");
    oled_line_append_str(oled_gas_string(peripheral_state_ptr->runtime.gas_detected));
    oled_write_line(2U, oled_line);

    oled_line_begin("SHT31:");
    oled_line_append_str(oled_sht31_status_string(peripheral_state_ptr->runtime.sht31_status));
    oled_write_line(3U, oled_line);

    oled_format_signed_centi(oled_value0,
                             sizeof(oled_value0),
                             peripheral_state_ptr->runtime.temperature_centi_c);
    oled_line_begin("T:");
    oled_line_append_str(oled_value0);
    oled_line_append_str("C");
    oled_write_line(4U, oled_line);

    oled_format_unsigned_centi(oled_value0,
                               sizeof(oled_value0),
                               peripheral_state_ptr->runtime.humidity_centi_pct);
    oled_line_begin("RH:");
    oled_line_append_str(oled_value0);
    oled_line_append_str("%");
    oled_write_line(5U, oled_line);
}

static void oled_render_page(uint8_t page)
{
    ssd1306_Fill(Black);

    switch (page) {
    case 0U:
        oled_render_summary();
        break;
    case 1U:
        oled_render_motion();
        break;
    case 2U:
        oled_render_arm();
        break;
    case 3U:
    default:
        oled_render_peripheral();
        break;
    }

    ssd1306_UpdateScreen();
}

void StartOledTask(void *argument)
{
    uint8_t page = 0U;

    (void)argument;

    ssd1306_Init();

    for (;;) {
        if (oled_key_next_page_event() != 0U) {
            page = (uint8_t)((page + 1U) % OLED_PAGE_COUNT);
        }

        oled_render_page(page);

        osDelay(OLED_TASK_PERIOD_MS);
    }
}
