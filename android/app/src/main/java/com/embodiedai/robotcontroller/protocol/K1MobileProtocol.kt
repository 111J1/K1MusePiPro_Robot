package com.embodiedai.robotcontroller.protocol

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.zip.CRC32
import java.util.zip.Inflater

enum class K1MessageType(val wireValue: Int) {
    HELLO(0x01),
    HEARTBEAT(0x02),
    MAP_INFO(0x10),
    MAP_TILE(0x11),
    ROBOT_POSE(0x20),
    TELEOP_CMD(0x30),
    STOP(0x31),
    NAV_PATH(0x43),
    BRIDGE_CONTROL(0x48),
    BRIDGE_STATUS(0x49),
    MAP_CONTROL(0x50),
    MAP_CONTROL_STATUS(0x51),
    MAP_LIBRARY_REQUEST(0x60),
    MAP_LIBRARY_STATUS(0x61),
    MAP_LIBRARY_LIST(0x62),
    MAP_REGIONS_DATA(0x63),
    ERROR(0x7F);

    companion object {
        fun fromWire(value: Int): K1MessageType? = entries.firstOrNull { it.wireValue == value }
    }
}

object K1MobileProtocol {
    fun bridgeControl(seq: Long, command: K1BridgeCommand): ByteArray {
        val payload = ByteBuffer.allocate(5)
            .order(ByteOrder.LITTLE_ENDIAN)
            .putInt((System.currentTimeMillis() and 0xffffffffL).toInt())
            .put(command.wireValue.toByte())
            .array()
        return frame(K1MessageType.BRIDGE_CONTROL, seq, payload)
    }

    fun mapControl(
        seq: Long,
        command: K1MapCommand,
        mode: K1MappingMode? = null,
        roomSize: K1AutoMapRoomSize? = null,
        customSizeX: Float? = null,
        customSizeY: Float? = null
    ): ByteArray {
        val includeOptions = mode != null || roomSize != null
        val includeCustomSize =
            mode == K1MappingMode.AUTO && roomSize == K1AutoMapRoomSize.CUSTOM
        val payload = ByteBuffer.allocate(
            if (includeCustomSize) 15 else if (includeOptions) 7 else 5
        )
            .order(ByteOrder.LITTLE_ENDIAN)
            .putInt((System.currentTimeMillis() and 0xffffffffL).toInt())
            .put(command.wireValue.toByte())
            .apply {
                if (includeOptions) {
                    put((mode ?: K1MappingMode.MANUAL).wireValue.toByte())
                    put((roomSize ?: K1AutoMapRoomSize.MEDIUM).wireValue.toByte())
                    if (includeCustomSize) {
                        putFloat(customSizeX ?: 10f)
                        putFloat(customSizeY ?: 10f)
                    }
                }
            }
            .array()
        return frame(K1MessageType.MAP_CONTROL, seq, payload)
    }

    fun mapLibraryRequest(
        seq: Long,
        command: K1MapLibraryCommand,
        mapName: String = "",
        regionsJson: String = ""
    ): ByteArray {
        val mapBytes = mapName.toByteArray(Charsets.UTF_8)
        val jsonBytes = regionsJson.toByteArray(Charsets.UTF_8)
        require(mapBytes.size <= U_SHORT_MAX) { "map name is too long" }
        require(jsonBytes.size <= U_SHORT_MAX) { "regions json is too long" }
        val payload = ByteBuffer.allocate(5 + 2 + mapBytes.size + 2 + jsonBytes.size)
            .order(ByteOrder.LITTLE_ENDIAN)
            .putInt((System.currentTimeMillis() and 0xffffffffL).toInt())
            .put(command.wireValue.toByte())
            .putShort(mapBytes.size.toShort())
            .put(mapBytes)
            .putShort(jsonBytes.size.toShort())
            .put(jsonBytes)
            .array()
        return frame(K1MessageType.MAP_LIBRARY_REQUEST, seq, payload)
    }

    fun teleop(seq: Long, vx: Float, vy: Float, omega: Float): ByteArray {
        val payload = ByteBuffer.allocate(16)
            .order(ByteOrder.LITTLE_ENDIAN)
            .putInt((System.currentTimeMillis() and 0xffffffffL).toInt())
            .putFloat(vx)
            .putFloat(vy)
            .putFloat(omega)
            .array()
        return frame(K1MessageType.TELEOP_CMD, seq, payload)
    }

    fun stop(seq: Long): ByteArray {
        val payload = ByteBuffer.allocate(4)
            .order(ByteOrder.LITTLE_ENDIAN)
            .putInt((System.currentTimeMillis() and 0xffffffffL).toInt())
            .array()
        return frame(K1MessageType.STOP, seq, payload)
    }

    private fun frame(type: K1MessageType, seq: Long, payload: ByteArray): ByteArray {
        val frame = ByteBuffer.allocate(HEADER_SIZE + payload.size)
            .order(ByteOrder.LITTLE_ENDIAN)
            .put(MAGIC)
            .put(VERSION.toByte())
            .put(type.wireValue.toByte())
            .putShort(0)
            .putInt(seq.toInt())
            .putInt(payload.size)
            .putInt(payloadCrc(payload).toInt())
            .put(payload)
        return frame.array()
    }

    private fun payloadCrc(payload: ByteArray): Long {
        val crc = CRC32()
        crc.update(payload)
        return crc.value
    }

    private val MAGIC = byteArrayOf('K'.code.toByte(), '1'.code.toByte(), 'M'.code.toByte(), 'B'.code.toByte())
    private const val VERSION = 1
    private const val HEADER_SIZE = 20
    private const val U_SHORT_MAX = 0xffff
}

data class K1Frame(
    val type: K1MessageType,
    val seq: Long,
    val payload: ByteArray
)

data class K1MapInfo(
    val mapVersion: Long,
    val width: Int,
    val height: Int,
    val resolution: Float,
    val originX: Float,
    val originY: Float,
    val originYaw: Float,
    val tileSize: Int
)

data class K1MapTile(
    val mapVersion: Long,
    val x: Int,
    val y: Int,
    val width: Int,
    val height: Int,
    val raw: ByteArray
)

data class K1MapLibraryEntry(
    val yamlName: String,
    val imageName: String,
    val hasRegions: Boolean
)

enum class K1MapLibraryCommand(val wireValue: Int) {
    LIST_MAPS(1),
    LOAD_MAP(2),
    READ_REGIONS(3),
    SAVE_REGIONS(4);

    companion object {
        fun fromWire(value: Int): K1MapLibraryCommand =
            entries.firstOrNull { it.wireValue == value } ?: LIST_MAPS
    }
}

data class K1MapLibraryStatus(
    val stampMs: Long,
    val command: K1MapLibraryCommand,
    val success: Boolean,
    val message: String
)

data class K1MapRegionsData(
    val stampMs: Long,
    val yamlName: String,
    val exists: Boolean,
    val json: String
)

data class K1RobotPose(
    val stampMs: Long,
    val x: Float,
    val y: Float,
    val yaw: Float
)

data class K1PathPoint(
    val x: Float,
    val y: Float
)

data class K1NavPath(
    val stampMs: Long,
    val pathId: Long,
    val points: List<K1PathPoint>
)

enum class K1BridgeCommand(val wireValue: Int) {
    START_BRIDGE(1),
    STOP_BRIDGE(2),
    QUERY_BRIDGE(3);

    companion object {
        fun fromWire(value: Int): K1BridgeCommand =
            entries.firstOrNull { it.wireValue == value } ?: QUERY_BRIDGE
    }
}

enum class K1BridgeState(val wireValue: Int) {
    SUPERVISOR(0),
    STARTING(1),
    ONLINE(2),
    STOPPING(3),
    ERROR(4);

    companion object {
        fun fromWire(value: Int): K1BridgeState =
            entries.firstOrNull { it.wireValue == value } ?: ERROR
    }
}

enum class K1MappingMode(val wireValue: Int) {
    MANUAL(0),
    AUTO(1)
}

enum class K1AutoMapRoomSize(val wireValue: Int, val label: String, val detail: String) {
    SMALL(0, "Small", "36 m2 | +/-3 m"),
    MEDIUM(1, "Medium", "100 m2 | +/-5 m"),
    LARGE(2, "Large", "225 m2 | +/-7.5 m"),
    CUSTOM(3, "Custom", "enter X/Y")
}

enum class K1MapCommand(val wireValue: Int) {
    START_MAPPING(1),
    SAVE_MAP_MANUAL(2),
    STOP_MAPPING(3),
    QUERY_MAPPING(4);

    companion object {
        fun fromWire(value: Int): K1MapCommand =
            entries.firstOrNull { it.wireValue == value } ?: QUERY_MAPPING
    }
}

enum class K1MapState(val wireValue: Int) {
    IDLE(0),
    STARTING(1),
    MAPPING(2),
    SAVING(3),
    STOPPING(4),
    ERROR(5);

    companion object {
        fun fromWire(value: Int): K1MapState =
            entries.firstOrNull { it.wireValue == value } ?: ERROR
    }
}

data class K1BridgeStatus(
    val stampMs: Long,
    val state: K1BridgeState,
    val command: K1BridgeCommand,
    val success: Boolean,
    val message: String
)

data class K1MapControlStatus(
    val stampMs: Long,
    val state: K1MapState,
    val command: K1MapCommand,
    val success: Boolean,
    val mapBase: String,
    val message: String
)

class K1FrameParser {
    // Ring buffer over a primitive byte array — avoids millions of boxed Byte
    // objects from ArrayList<Byte>, which caused multi-second GC pauses during
    // map streaming and delayed teleop processing.
    private var buf = ByteArray(65536)
    private var head = 0
    private var count = 0

    fun feed(data: ByteArray, length: Int, onFrame: (K1Frame) -> Unit) {
        val needed = count + length
        if (needed > buf.size && needed <= MAX_PAYLOAD_SIZE + HEADER_SIZE) {
            val next = ByteArray(minOf(needed * 2, MAX_PAYLOAD_SIZE + HEADER_SIZE))
            if (count > 0) {
                val h = head
                val firstChunk = minOf(count, buf.size - h)
                System.arraycopy(buf, h, next, 0, firstChunk)
                if (firstChunk < count) {
                    System.arraycopy(buf, 0, next, firstChunk, count - firstChunk)
                }
            }
            buf = next
            head = 0
        } else if (needed > MAX_PAYLOAD_SIZE + HEADER_SIZE) {
            // overflow: discard everything and start fresh
            head = 0
            count = 0
        }

        // Append new bytes to the ring buffer
        for (i in 0 until length) {
            val pos = (head + count) % buf.size
            buf[pos] = data[i]
            count++
        }
        parse(onFrame)
    }

    // Discard n bytes from the front of the ring buffer
    private fun discard(n: Int) {
        head = (head + n) % buf.size
        count -= n
    }

    // Read byte at offset 'i' from the current head
    private fun byteAt(i: Int): Byte = buf[(head + i) % buf.size]

    private fun regionEquals(start: Int, needle: ByteArray): Boolean {
        if (start + needle.size > count) return false
        for (j in needle.indices) {
            if (byteAt(start + j) != needle[j]) return false
        }
        return true
    }

    private fun copyRange(from: Int, to: Int): ByteArray {
        val len = to - from
        val result = ByteArray(len)
        val h = (head + from) % buf.size
        val firstChunk = minOf(len, buf.size - h)
        System.arraycopy(buf, h, result, 0, firstChunk)
        if (firstChunk < len) {
            System.arraycopy(buf, 0, result, firstChunk, len - firstChunk)
        }
        return result
    }

    private fun parse(onFrame: (K1Frame) -> Unit) {
        while (true) {
            val start = findMagic()
            if (start < 0) {
                // Keep the last 3 bytes in case magic is split across reads
                if (count > 3) {
                    val excess = count - 3
                    discard(excess)
                }
                return
            }
            if (start > 0) {
                discard(start)
            }
            if (count < HEADER_SIZE) {
                return
            }

            val header = copyRange(0, HEADER_SIZE)
            val version = header[4].toInt() and 0xff
            val typeValue = header[5].toInt() and 0xff
            val seq = header.u32At(8)
            val len = header.u32At(12)
            val expectedCrc = header.u32At(16)

            if (version != VERSION || len > MAX_PAYLOAD_SIZE) {
                discard(1) // skip one byte and retry
                continue
            }
            val total = HEADER_SIZE + len.toInt()
            if (count < total) {
                return
            }

            val payload = copyRange(HEADER_SIZE, total)
            discard(total)

            val crc = CRC32()
            crc.update(payload)
            if (crc.value != expectedCrc) {
                continue
            }
            val type = K1MessageType.fromWire(typeValue) ?: continue
            onFrame(K1Frame(type, seq, payload))
        }
    }

    private fun findMagic(): Int {
        var i = 0
        val limit = count - MAGIC.size
        while (i <= limit) {
            if (byteAt(i) == MAGIC[0] && regionEquals(i, MAGIC)) {
                return i
            }
            // Skip ahead using the next possible 'K' position
            val nextK = nextMagicCandidate(i + 1, limit)
            i = if (nextK > i) nextK else i + 1
        }
        return -1
    }

    // Find the next occurrence of the first magic byte ('K')
    private fun nextMagicCandidate(from: Int, limit: Int): Int {
        for (i in from..limit) {
            if (byteAt(i) == MAGIC[0]) return i
        }
        return limit + 1
    }

    private companion object {
        val MAGIC = byteArrayOf('K'.code.toByte(), '1'.code.toByte(), 'M'.code.toByte(), 'B'.code.toByte())
        const val VERSION = 1
        const val HEADER_SIZE = 20
        const val MAX_PAYLOAD_SIZE = 4 * 1024 * 1024
    }
}

fun K1Frame.decodeMapInfo(): K1MapInfo? {
    if (type != K1MessageType.MAP_INFO || payload.size < 30) {
        return null
    }
    val bb = payloadBuffer()
    return K1MapInfo(
        mapVersion = bb.u32(),
        width = bb.u32().toInt(),
        height = bb.u32().toInt(),
        resolution = bb.float,
        originX = bb.float,
        originY = bb.float,
        originYaw = bb.float,
        tileSize = bb.u16()
    )
}

fun K1Frame.decodeMapTile(): K1MapTile? {
    if (type != K1MessageType.MAP_TILE || payload.size < 18) {
        return null
    }
    val bb = payloadBuffer()
    val mapVersion = bb.u32()
    val x = bb.u32().toInt()
    val y = bb.u32().toInt()
    val width = bb.u16()
    val height = bb.u16()
    val rawLen = bb.u32().toInt()
    val encoding = bb.get().toInt() and 0xff
    val encoded = ByteArray(bb.remaining())
    bb.get(encoded)
    val raw = when (encoding) {
        TILE_ENCODING_RAW -> encoded
        TILE_ENCODING_ZLIB -> inflate(encoded, rawLen)
        else -> return null
    }
    if (raw.size != rawLen || raw.size != width * height) {
        return null
    }
    return K1MapTile(mapVersion, x, y, width, height, raw)
}

fun K1Frame.decodeRobotPose(): K1RobotPose? {
    if (type != K1MessageType.ROBOT_POSE || payload.size < 16) {
        return null
    }
    val bb = payloadBuffer()
    return K1RobotPose(
        stampMs = bb.u32(),
        x = bb.float,
        y = bb.float,
        yaw = bb.float
    )
}

fun K1Frame.decodeNavPath(): K1NavPath? {
    if (type != K1MessageType.NAV_PATH || payload.size < 10) {
        return null
    }
    val bb = payloadBuffer()
    val stampMs = bb.u32()
    val pathId = bb.u32()
    val count = bb.u16()
    if (count < 2 || bb.remaining() < count * 8) {
        return null
    }
    val points = ArrayList<K1PathPoint>(count)
    repeat(count) {
        points.add(K1PathPoint(bb.float, bb.float))
    }
    return K1NavPath(stampMs, pathId, points)
}

fun K1Frame.decodeHelloName(): String? {
    if (type != K1MessageType.HELLO || payload.size < 2) {
        return null
    }
    val bb = payloadBuffer()
    return bb.readString()
}

fun K1Frame.decodeBridgeStatus(): K1BridgeStatus? {
    if (type != K1MessageType.BRIDGE_STATUS || payload.size < 9) {
        return null
    }
    val bb = payloadBuffer()
    return K1BridgeStatus(
        stampMs = bb.u32(),
        state = K1BridgeState.fromWire(bb.get().toInt() and 0xff),
        command = K1BridgeCommand.fromWire(bb.get().toInt() and 0xff),
        success = (bb.get().toInt() and 0xff) != 0,
        message = bb.readString().orEmpty()
    )
}

fun K1Frame.decodeMapControlStatus(): K1MapControlStatus? {
    if (type != K1MessageType.MAP_CONTROL_STATUS || payload.size < 11) {
        return null
    }
    val bb = payloadBuffer()
    return K1MapControlStatus(
        stampMs = bb.u32(),
        state = K1MapState.fromWire(bb.get().toInt() and 0xff),
        command = K1MapCommand.fromWire(bb.get().toInt() and 0xff),
        success = (bb.get().toInt() and 0xff) != 0,
        mapBase = bb.readString().orEmpty(),
        message = bb.readString().orEmpty()
    )
}

fun K1Frame.decodeMapLibraryStatus(): K1MapLibraryStatus? {
    if (type != K1MessageType.MAP_LIBRARY_STATUS || payload.size < 8) {
        return null
    }
    val bb = payloadBuffer()
    return K1MapLibraryStatus(
        stampMs = bb.u32(),
        command = K1MapLibraryCommand.fromWire(bb.get().toInt() and 0xff),
        success = (bb.get().toInt() and 0xff) != 0,
        message = bb.readString().orEmpty()
    )
}

fun K1Frame.decodeMapLibraryList(): List<K1MapLibraryEntry>? {
    if (type != K1MessageType.MAP_LIBRARY_LIST || payload.size < 6) {
        return null
    }
    val bb = payloadBuffer()
    bb.u32()
    val count = bb.u16()
    val entries = ArrayList<K1MapLibraryEntry>(count)
    repeat(count) {
        val yamlName = bb.readString() ?: return null
        val imageName = bb.readString() ?: return null
        if (bb.remaining() < 1) {
            return null
        }
        entries.add(
            K1MapLibraryEntry(
                yamlName = yamlName,
                imageName = imageName,
                hasRegions = (bb.get().toInt() and 0xff) != 0
            )
        )
    }
    return entries
}

fun K1Frame.decodeMapRegionsData(): K1MapRegionsData? {
    if (type != K1MessageType.MAP_REGIONS_DATA || payload.size < 7) {
        return null
    }
    val bb = payloadBuffer()
    val stamp = bb.u32()
    val yamlName = bb.readString() ?: return null
    if (bb.remaining() < 1) {
        return null
    }
    val exists = (bb.get().toInt() and 0xff) != 0
    val json = bb.readString() ?: return null
    return K1MapRegionsData(stamp, yamlName, exists, json)
}

private fun K1Frame.payloadBuffer(): ByteBuffer {
    return ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)
}

private fun ByteArray.u32At(offset: Int): Long {
    return ((this[offset].toLong() and 0xffL) or
        ((this[offset + 1].toLong() and 0xffL) shl 8) or
        ((this[offset + 2].toLong() and 0xffL) shl 16) or
        ((this[offset + 3].toLong() and 0xffL) shl 24))
}

private fun ByteBuffer.u32(): Long {
    return int.toLong() and 0xffffffffL
}

private fun ByteBuffer.u16(): Int {
    return short.toInt() and 0xffff
}

private fun ByteBuffer.readString(): String? {
    if (remaining() < 2) {
        return null
    }
    val len = u16()
    if (len > remaining()) {
        return null
    }
    val raw = ByteArray(len)
    get(raw)
    return raw.toString(Charsets.UTF_8)
}

private fun inflate(encoded: ByteArray, rawLen: Int): ByteArray {
    val inflater = Inflater()
    return try {
        inflater.setInput(encoded)
        val raw = ByteArray(rawLen)
        val count = inflater.inflate(raw)
        if (count == rawLen) raw else ByteArray(0)
    } finally {
        inflater.end()
    }
}

private const val TILE_ENCODING_RAW = 0
private const val TILE_ENCODING_ZLIB = 1
