#include "k1muse_control_core/control_protocol.h"

#include <string.h>

static void parser_reset(k1_ctrl_parser_t *parser)
{
    parser->state = K1_CTRL_PARSE_WAIT_SOF1;
    parser->header_index = 0U;
    parser->payload_index = 0U;
}

void k1_ctrl_parser_init(k1_ctrl_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser_reset(parser);
}

int k1_ctrl_parser_input(k1_ctrl_parser_t *parser, uint8_t byte,
                         k1_ctrl_frame_t *frame_out)
{
    uint8_t crc_buffer[5U + K1_CTRL_MAX_PAYLOAD];
    uint8_t expected_crc;

    if ((parser == NULL) || (frame_out == NULL)) {
        return 0;
    }

    switch (parser->state) {
    case K1_CTRL_PARSE_WAIT_SOF1:
        if (byte == K1_CTRL_SOF1) {
            parser->state = K1_CTRL_PARSE_WAIT_SOF2;
        }
        break;

    case K1_CTRL_PARSE_WAIT_SOF2:
        if (byte == K1_CTRL_SOF2) {
            parser->state = K1_CTRL_PARSE_HEADER;
            parser->header_index = 0U;
        } else if (byte != K1_CTRL_SOF1) {
            parser_reset(parser);
        }
        break;

    case K1_CTRL_PARSE_HEADER:
        parser->header[parser->header_index++] = byte;
        if (parser->header_index == 5U) {
            parser->frame.src = parser->header[0];
            parser->frame.target = parser->header[1];
            parser->frame.cmd = parser->header[2];
            parser->frame.seq = parser->header[3];
            parser->frame.len = parser->header[4];
            if (parser->frame.len > K1_CTRL_MAX_PAYLOAD) {
                parser_reset(parser);
            } else if (parser->frame.len == 0U) {
                parser->state = K1_CTRL_PARSE_CRC;
            } else {
                parser->payload_index = 0U;
                parser->state = K1_CTRL_PARSE_PAYLOAD;
            }
        }
        break;

    case K1_CTRL_PARSE_PAYLOAD:
        parser->frame.payload[parser->payload_index++] = byte;
        if (parser->payload_index == parser->frame.len) {
            parser->state = K1_CTRL_PARSE_CRC;
        }
        break;

    case K1_CTRL_PARSE_CRC:
        memcpy(crc_buffer, parser->header, 5U);
        if (parser->frame.len > 0U) {
            memcpy(&crc_buffer[5], parser->frame.payload, parser->frame.len);
        }
        expected_crc = k1_crc8_atm(crc_buffer, 5U + parser->frame.len);
        if (byte == expected_crc) {
            *frame_out = parser->frame;
            parser_reset(parser);
            return 1;
        }
        parser_reset(parser);
        break;

    default:
        parser_reset(parser);
        break;
    }

    return 0;
}

size_t k1_ctrl_parser_size(void)
{
    return sizeof(k1_ctrl_parser_t);
}

size_t k1_ctrl_encode_frame(const k1_ctrl_frame_t *frame,
                            uint8_t *output, size_t output_size)
{
    size_t frame_size;

    if ((frame == NULL) || (output == NULL) ||
        (frame->len > K1_CTRL_MAX_PAYLOAD)) {
        return 0U;
    }

    frame_size = 8U + frame->len;
    if (output_size < frame_size) {
        return 0U;
    }

    output[0] = K1_CTRL_SOF1;
    output[1] = K1_CTRL_SOF2;
    output[2] = frame->src;
    output[3] = frame->target;
    output[4] = frame->cmd;
    output[5] = frame->seq;
    output[6] = frame->len;
    if (frame->len > 0U) {
        memcpy(&output[7], frame->payload, frame->len);
    }
    output[7U + frame->len] = k1_crc8_atm(&output[2], 5U + frame->len);
    return frame_size;
}

void k1_put_u32_le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8U) & 0xFFU);
    output[2] = (uint8_t)((value >> 16U) & 0xFFU);
    output[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

uint32_t k1_get_u32_le(const uint8_t *input)
{
    return (uint32_t)input[0] |
           ((uint32_t)input[1] << 8U) |
           ((uint32_t)input[2] << 16U) |
           ((uint32_t)input[3] << 24U);
}

void k1_put_f32_le(uint8_t *output, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    k1_put_u32_le(output, bits);
}

float k1_get_f32_le(const uint8_t *input)
{
    uint32_t bits = k1_get_u32_le(input);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}
