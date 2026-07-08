#ifndef K1MUSE_CONTROL_CORE__CONTROL_PROTOCOL_H_
#define K1MUSE_CONTROL_CORE__CONTROL_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define K1_CTRL_SOF1 (0xA5U)
#define K1_CTRL_SOF2 (0x5AU)
#define K1_CTRL_MAX_PAYLOAD (64U)
#define K1_CTRL_MAX_FRAME_SIZE (72U)

typedef enum {
    K1_CTRL_PARSE_WAIT_SOF1 = 0,
    K1_CTRL_PARSE_WAIT_SOF2,
    K1_CTRL_PARSE_HEADER,
    K1_CTRL_PARSE_PAYLOAD,
    K1_CTRL_PARSE_CRC,
} k1_ctrl_parse_state_t;

typedef struct {
    uint8_t src;
    uint8_t target;
    uint8_t cmd;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[K1_CTRL_MAX_PAYLOAD];
} k1_ctrl_frame_t;

typedef struct {
    k1_ctrl_parse_state_t state;
    k1_ctrl_frame_t frame;
    uint8_t header[5];
    uint8_t header_index;
    uint8_t payload_index;
} k1_ctrl_parser_t;

uint8_t k1_crc8_atm(const uint8_t *data, size_t len);

void k1_ctrl_parser_init(k1_ctrl_parser_t *parser);
int k1_ctrl_parser_input(k1_ctrl_parser_t *parser, uint8_t byte,
                         k1_ctrl_frame_t *frame_out);
size_t k1_ctrl_parser_size(void);

size_t k1_ctrl_encode_frame(const k1_ctrl_frame_t *frame,
                            uint8_t *output, size_t output_size);

void k1_put_u32_le(uint8_t *output, uint32_t value);
uint32_t k1_get_u32_le(const uint8_t *input);
void k1_put_f32_le(uint8_t *output, float value);
float k1_get_f32_le(const uint8_t *input);

#ifdef __cplusplus
}
#endif

#endif
