package com.embodiedai.robotcontroller.protocol

import java.nio.ByteBuffer
import java.nio.ByteOrder

enum class CoordinateMode(val wireValue: Byte) {
    LCS(0x00),
    WCS(0x01)
}

object ControlProtocol {
    private const val SOF1: Byte = 0xA5.toByte()
    private const val SOF2: Byte = 0x5A.toByte()
    private const val SRC_BT: Byte = 0x01
    private const val TARGET_CHASSIS: Byte = 0x01
    private const val CMD_STOP: Byte = 0x00
    private const val CMD_MOV: Byte = 0x01
    private const val CMD_ODOM: Byte = 0x02

    const val MAX_CHASSIS_V = 1.4f
    const val MAX_CHASSIS_OMEGA = 3.7f

    fun stop(seq: Int): ByteArray {
        return frame(CMD_STOP, seq, ByteArray(0))
    }

    fun move(
        seq: Int,
        mode: CoordinateMode,
        direction: Float,
        velocity: Float,
        omega: Float
    ): ByteArray {
        val payload = ByteBuffer.allocate(13)
            .order(ByteOrder.LITTLE_ENDIAN)
            .put(mode.wireValue)
            .putFloat(direction)
            .putFloat(velocity)
            .putFloat(omega)
            .array()

        return frame(CMD_MOV, seq, payload)
    }

    fun resetOdometry(seq: Int): ByteArray {
        val payload = ByteBuffer.allocate(12)
            .order(ByteOrder.LITTLE_ENDIAN)
            .putFloat(0f)
            .putFloat(0f)
            .putFloat(0f)
            .array()

        return frame(CMD_ODOM, seq, payload)
    }

    private fun frame(cmd: Byte, seq: Int, payload: ByteArray): ByteArray {
        val len = payload.size
        val frame = ByteArray(2 + 5 + len + 1)
        var index = 0

        frame[index++] = SOF1
        frame[index++] = SOF2
        frame[index++] = SRC_BT
        frame[index++] = TARGET_CHASSIS
        frame[index++] = cmd
        frame[index++] = seq.toByte()
        frame[index++] = len.toByte()
        payload.copyInto(frame, index)
        index += len
        frame[index] = crc8Atm(frame, offset = 2, length = 5 + len)

        return frame
    }

    private fun crc8Atm(data: ByteArray, offset: Int, length: Int): Byte {
        var crc = 0
        for (i in offset until offset + length) {
            crc = crc xor (data[i].toInt() and 0xff)
            repeat(8) {
                crc = if ((crc and 0x80) != 0) {
                    ((crc shl 1) xor 0x07) and 0xff
                } else {
                    (crc shl 1) and 0xff
                }
            }
        }
        return crc.toByte()
    }
}
