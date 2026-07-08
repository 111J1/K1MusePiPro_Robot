#include "mdl_control_protocol.h"

#include "alg_crc8.h"
#include <string.h>

static void control_protocol_reset(control_protocol_t *protocol)
{
    /* Reset to frame search state after a complete frame or parse error. */
    protocol->state = CONTROL_PROTOCOL_WAIT_SOF1;
    protocol->header_index = 0U;
    protocol->payload_index = 0U;
    memset(&protocol->frame, 0, sizeof(protocol->frame));
}

static uint8_t control_protocol_check_crc(const ctrl_frame_t *frame, uint8_t rx_crc)
{
    /* CRC covers SRC..PAYLOAD, excluding SOF bytes and the CRC byte itself. */
    uint8_t crc_buf[5U + CTRL_PROTOCOL_MAX_PAYLOAD];
    uint16_t crc_len = 0U;

    crc_buf[crc_len++] = frame->src;
    crc_buf[crc_len++] = frame->target;
    crc_buf[crc_len++] = frame->cmd;
    crc_buf[crc_len++] = frame->seq;
    crc_buf[crc_len++] = frame->len;

    if (frame->len > 0U) {
        memcpy(&crc_buf[crc_len], frame->payload, frame->len);
        crc_len = (uint16_t)(crc_len + frame->len);
    }

    return (alg_crc8_atm(crc_buf, crc_len) == rx_crc) ? 1U : 0U;
}

void control_protocol_init(control_protocol_t *protocol)
{
    if (protocol != 0) {
        control_protocol_reset(protocol);
    }
}

uint8_t control_protocol_input_byte(control_protocol_t *protocol, uint8_t byte,
                                    ctrl_frame_t *frame)
{
    if ((protocol == 0) || (frame == 0)) {
        return 0U;
    }

    switch (protocol->state) {
    case CONTROL_PROTOCOL_WAIT_SOF1:
        /* Keep scanning until the first sync byte appears. */
        if (byte == CTRL_FRAME_SOF1) {
            protocol->state = CONTROL_PROTOCOL_WAIT_SOF2;
        }
        break;

    case CONTROL_PROTOCOL_WAIT_SOF2:
        /* A repeated SOF1 may be the start of a new frame. */
        if (byte == CTRL_FRAME_SOF2) {
            protocol->state = CONTROL_PROTOCOL_READ_HEADER;
            protocol->header_index = 0U;
        } else if (byte != CTRL_FRAME_SOF1) {
            protocol->state = CONTROL_PROTOCOL_WAIT_SOF1;
        }
        break;

    case CONTROL_PROTOCOL_READ_HEADER:
        protocol->header_buf[protocol->header_index++] = byte;
        if (protocol->header_index >= sizeof(protocol->header_buf)) {
            protocol->frame.src = protocol->header_buf[0];
            protocol->frame.target = protocol->header_buf[1];
            protocol->frame.cmd = protocol->header_buf[2];
            protocol->frame.seq = protocol->header_buf[3];
            protocol->frame.len = protocol->header_buf[4];

            if (protocol->frame.len > CTRL_PROTOCOL_MAX_PAYLOAD) {
                /* Invalid length means the stream is out of sync. */
                control_protocol_reset(protocol);
            } else if (protocol->frame.len == 0U) {
                protocol->state = CONTROL_PROTOCOL_READ_CRC;
            } else {
                protocol->payload_index = 0U;
                protocol->state = CONTROL_PROTOCOL_READ_PAYLOAD;
            }
        }
        break;

    case CONTROL_PROTOCOL_READ_PAYLOAD:
        protocol->frame.payload[protocol->payload_index++] = byte;
        if (protocol->payload_index >= protocol->frame.len) {
            protocol->state = CONTROL_PROTOCOL_READ_CRC;
        }
        break;

    case CONTROL_PROTOCOL_READ_CRC:
        if (control_protocol_check_crc(&protocol->frame, byte) != 0U) {
            /* Copy the verified frame out before resetting parser state. */
            *frame = protocol->frame;
            control_protocol_reset(protocol);
            return 1U;
        }
        control_protocol_reset(protocol);
        break;

    default:
        control_protocol_reset(protocol);
        break;
    }

    return 0U;
}

uint8_t control_protocol_input_buffer(control_protocol_t *protocol,
                                      const uint8_t *data, uint16_t len,
                                      ctrl_frame_t *frame)
{
    uint16_t i;

    if ((protocol == 0) || (data == 0) || (frame == 0)) {
        return 0U;
    }

    for (i = 0U; i < len; ++i) {
        if (control_protocol_input_byte(protocol, data[i], frame) != 0U) {
            return 1U;
        }
    }

    return 0U;
}

uint16_t control_protocol_encode_frame(uint8_t src,
                                       uint8_t target,
                                       uint8_t cmd,
                                       uint8_t seq,
                                       const uint8_t *payload,
                                       uint8_t len,
                                       uint8_t *out_buf,
                                       uint16_t out_size)
{
    uint16_t frame_len;
    uint16_t crc_len;

    if ((out_buf == 0) || (len > CTRL_PROTOCOL_MAX_PAYLOAD)) {
        return 0U;
    }
    if ((len > 0U) && (payload == 0)) {
        return 0U;
    }

    frame_len = (uint16_t)(2U + 5U + len + 1U);
    if (out_size < frame_len) {
        return 0U;
    }

    out_buf[0] = CTRL_FRAME_SOF1;
    out_buf[1] = CTRL_FRAME_SOF2;
    out_buf[2] = src;
    out_buf[3] = target;
    out_buf[4] = cmd;
    out_buf[5] = seq;
    out_buf[6] = len;

    if (len > 0U) {
        memcpy(&out_buf[7], payload, len);
    }

    crc_len = (uint16_t)(5U + len);
    out_buf[frame_len - 1U] = alg_crc8_atm(&out_buf[2], crc_len);

    return frame_len;
}
