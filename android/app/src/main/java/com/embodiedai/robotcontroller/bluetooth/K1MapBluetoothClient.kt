package com.embodiedai.robotcontroller.bluetooth

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import androidx.core.content.ContextCompat
import com.embodiedai.robotcontroller.protocol.K1Frame
import com.embodiedai.robotcontroller.protocol.K1FrameParser
import com.embodiedai.robotcontroller.protocol.K1MessageType
import java.io.File
import java.io.IOException
import java.util.UUID
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.Executors

class K1MapBluetoothClient(private val context: Context) {
    private companion object {
        private const val TAG = "K1MapBluetoothClient"
        private const val RFCOMM_FALLBACK_CHANNEL = 1
        private const val PAIRING_WAIT_MS = 4000L
    }

    private val adapter: BluetoothAdapter? =
        context.getSystemService(BluetoothManager::class.java)?.adapter
    private val mainHandler = Handler(Looper.getMainLooper())
    private val ioExecutor = Executors.newSingleThreadExecutor()
    private val writeLock = Any()
    private val debugLock = Any()
    // Bounded queue of size 1: newest teleop frame replaces any unsent one,
    // preventing the 3-10s build-up when the Bluetooth socket write blocks.
    private val teleopQueue = ArrayBlockingQueue<ByteArray?>(1)
    private val readExecutor = Executors.newSingleThreadExecutor()
    private val teleopThread = Thread({
        try {
            while (true) {
                val frame = teleopQueue.take() ?: continue
                val target = currentSocket() ?: continue
                try {
                    val started = System.currentTimeMillis()
                    synchronized(writeLock) {
                        target.outputStream.write(frame)
                    }
                    debugLog(
                        "teleop_write",
                        "bytes=${frame.size}\tduration_ms=${System.currentTimeMillis() - started}"
                    )
                } catch (_: IOException) {
                    debugLog("teleop_write_fail", "bytes=${frame.size}")
                    closeSocketIfCurrent(target)
                }
            }
        } catch (_: InterruptedException) {
            Thread.currentThread().interrupt()
        }
    }, "K1-teleop-writer").apply {
        isDaemon = true
        start()
    }
    private val sppUuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
    private val stateLock = Any()

    @Volatile
    private var socket: BluetoothSocket? = null
    private var connectionToken = 0

    fun hasConnectPermission(): Boolean {
        return Build.VERSION.SDK_INT < Build.VERSION_CODES.S ||
            ContextCompat.checkSelfPermission(
                context,
                Manifest.permission.BLUETOOTH_CONNECT
            ) == PackageManager.PERMISSION_GRANTED
    }

    @SuppressLint("MissingPermission")
    fun connect(
        address: String,
        onFrame: (K1Frame) -> Unit,
        onResult: (Boolean, String) -> Unit,
        onDisconnected: (String) -> Unit
    ) {
        if (!hasConnectPermission()) {
            post { onResult(false, "Bluetooth permission is not granted") }
            return
        }
        val localAdapter = adapter
        if (localAdapter == null || !localAdapter.isEnabled) {
            post { onResult(false, "Bluetooth is disabled") }
            return
        }

        val token = nextToken()
        closeSocket()

        ioExecutor.execute {
            try {
                val bonded = localAdapter.bondedDevices?.any { it.address == address } == true
                if (!bonded) {
                    post { onResult(false, "Pair K1 in Android Bluetooth settings first") }
                    return@execute
                }
                val device = localAdapter.getRemoteDevice(address)
                localAdapter.cancelDiscovery()

                val newSocket = connectWithFallback(device, token)
                if (!setSocket(newSocket, token)) {
                    closeQuietly(newSocket)
                    post { onResult(false, "Connection was cancelled") }
                    return@execute
                }
                startReadPump(newSocket, token, onFrame, onDisconnected)
                post {
                    if (isTokenCurrent(token)) {
                        onResult(true, "Connected to ${device.name ?: address}")
                    } else {
                        onResult(false, "Connection was cancelled")
                    }
                }
            } catch (error: IOException) {
                closeSocket()
                post { onResult(false, error.message ?: "Connection failed") }
            } catch (error: IllegalArgumentException) {
                closeSocket()
                post { onResult(false, error.message ?: "Invalid Bluetooth address") }
            } catch (error: SecurityException) {
                closeSocket()
                post { onResult(false, error.message ?: "Bluetooth permission is not granted") }
            }
        }
    }

    fun disconnect(onDone: (() -> Unit)? = null) {
        nextToken()
        closeSocket()
        if (onDone != null) {
            post(onDone)
        }
    }

    // Dedicated teleop sender — uses a bounded queue (size 1) so the
    // latest frame always replaces any stale unsent one.  No queue build-up.
    fun sendTeleopAsync(data: ByteArray) {
        // poll() drains any un-sent frame, then offer() inserts the new one
        val dropped = teleopQueue.poll() != null
        teleopQueue.offer(data)
        debugLog("teleop_enqueue", "bytes=${data.size}\tdropped=${if (dropped) 1 else 0}")
    }

    fun sendAsync(data: ByteArray, onFailure: (String) -> Unit = {}) {
        ioExecutor.execute {
            val target = currentSocket()
            if (target == null) {
                post { onFailure("K1 Bluetooth is not connected") }
                return@execute
            }
            try {
                val started = System.currentTimeMillis()
                synchronized(writeLock) {
                    target.outputStream.write(data)
                }
                debugLog(
                    "send_async",
                    "type=${frameTypeName(data)}\tbytes=${data.size}\tduration_ms=${System.currentTimeMillis() - started}"
                )
            } catch (error: IOException) {
                debugLog("send_async_fail", "type=${frameTypeName(data)}\tbytes=${data.size}")
                closeSocketIfCurrent(target)
                post { onFailure(error.message ?: "K1 Bluetooth send failed") }
            }
        }
    }

    private fun startReadPump(
        activeSocket: BluetoothSocket,
        token: Int,
        onFrame: (K1Frame) -> Unit,
        onDisconnected: (String) -> Unit
    ) {
        readExecutor.execute {
            val parser = K1FrameParser()
            val buffer = ByteArray(1024)
            try {
                while (isCurrent(activeSocket, token)) {
                    val len = activeSocket.inputStream.read(buffer)
                    if (len < 0) {
                        if (isCurrent(activeSocket, token)) {
                            post { onDisconnected("Remote device disconnected") }
                        }
                        break
                    }
                    parser.feed(buffer, len) { frame ->
                        if (isCurrent(activeSocket, token)) {
                            val rxMs = System.currentTimeMillis()
                            debugLog("frame_rx", "type=${frame.type.name}\tpayload=${frame.payload.size}")
                            post {
                                val dispatchMs = System.currentTimeMillis()
                                val started = dispatchMs
                                onFrame(frame)
                                debugLog(
                                    "frame_main",
                                    "type=${frame.type.name}\tlag_ms=${dispatchMs - rxMs}\tduration_ms=${System.currentTimeMillis() - started}"
                                )
                            }
                        }
                    }
                }
            } catch (error: IOException) {
                if (isCurrent(activeSocket, token)) {
                    post { onDisconnected(error.message ?: "Disconnected") }
                }
            } finally {
                closeSocketIfCurrent(activeSocket)
            }
        }
    }

    private fun connectWithFallback(device: BluetoothDevice, token: Int): BluetoothSocket {
        val failures = mutableListOf<String>()
        val attempts = listOf(
            SocketAttempt("RFCOMM channel $RFCOMM_FALLBACK_CHANNEL") {
                createRfcommSocket(device, RFCOMM_FALLBACK_CHANNEL)
            },
            SocketAttempt("Insecure RFCOMM channel $RFCOMM_FALLBACK_CHANNEL") {
                createInsecureRfcommSocket(device, RFCOMM_FALLBACK_CHANNEL)
            },
            SocketAttempt("SPP UUID") {
                device.createRfcommSocketToServiceRecord(sppUuid)
            },
            SocketAttempt("Insecure SPP UUID") {
                device.createInsecureRfcommSocketToServiceRecord(sppUuid)
            }
        )

        for (pass in 0..1) {
            for (attempt in attempts) {
                if (!isTokenCurrent(token)) {
                    throw IOException("Connection was cancelled")
                }

                val candidate = try {
                    attempt.create()
                } catch (error: Exception) {
                    failures += "${attempt.label}: ${error.message ?: error.javaClass.simpleName}"
                    Log.w(TAG, "Failed to create ${attempt.label} socket", error)
                    null
                }

                if (candidate == null) {
                    continue
                }

                try {
                    candidate.connect()
                    return candidate
                } catch (error: IOException) {
                    failures += "${attempt.label}: ${error.message ?: "connect failed"}"
                    Log.w(TAG, "Failed to connect with ${attempt.label}", error)
                    closeQuietly(candidate)
                }
            }

            if (pass == 0) {
                val waitStart = System.currentTimeMillis()
                while (System.currentTimeMillis() - waitStart < PAIRING_WAIT_MS) {
                    if (!isTokenCurrent(token)) {
                        throw IOException("Connection was cancelled")
                    }
                    Thread.sleep(200)
                }
                failures.add("--- retry after pairing wait ---")
            }
        }

        throw IOException(failures.joinToString(separator = "; ").ifBlank { "Connection failed" })
    }

    private fun createRfcommSocket(device: BluetoothDevice, channel: Int): BluetoothSocket? {
        val method = device.javaClass.getMethod(
            "createRfcommSocket",
            Int::class.javaPrimitiveType
        )
        return method.invoke(device, channel) as? BluetoothSocket
    }

    private fun createInsecureRfcommSocket(device: BluetoothDevice, channel: Int): BluetoothSocket? {
        val method = device.javaClass.getMethod(
            "createInsecureRfcommSocket",
            Int::class.javaPrimitiveType
        )
        return method.invoke(device, channel) as? BluetoothSocket
    }

    private fun nextToken(): Int {
        synchronized(stateLock) {
            connectionToken += 1
            return connectionToken
        }
    }

    private fun isTokenCurrent(token: Int): Boolean {
        synchronized(stateLock) {
            return connectionToken == token
        }
    }

    private fun setSocket(newSocket: BluetoothSocket, token: Int): Boolean {
        synchronized(stateLock) {
            if (token != connectionToken) {
                return false
            }
            socket = newSocket
            return true
        }
    }

    private fun isCurrent(activeSocket: BluetoothSocket, token: Int): Boolean {
        synchronized(stateLock) {
            return socket === activeSocket && connectionToken == token
        }
    }

    private fun closeSocket() {
        val target: BluetoothSocket?
        synchronized(stateLock) {
            target = socket
            socket = null
        }
        closeQuietly(target)
    }

    private fun currentSocket(): BluetoothSocket? {
        synchronized(stateLock) {
            return socket
        }
    }

    private fun closeSocketIfCurrent(target: BluetoothSocket?) {
        if (target == null) {
            return
        }
        synchronized(stateLock) {
            if (socket !== target) {
                return
            }
            socket = null
        }
        closeQuietly(target)
    }

    private fun closeQuietly(target: BluetoothSocket?) {
        try {
            target?.close()
        } catch (_: IOException) {
        }
    }

    private fun post(block: () -> Unit) {
        mainHandler.post(block)
    }

    private fun debugLog(event: String, fields: String) {
        synchronized(debugLock) {
            val dir = File(context.filesDir, "k1_debug")
            dir.mkdirs()
            File(dir, "bluetooth_debug.tsv").appendText(
                "${System.currentTimeMillis()}\t$event\t$fields\n"
            )
        }
    }

    private fun frameTypeName(data: ByteArray): String {
        val raw = data.getOrNull(5)?.toInt()?.and(0xff) ?: return "unknown"
        return K1MessageType.fromWire(raw)?.name ?: "0x${raw.toString(16)}"
    }

    private data class SocketAttempt(
        val label: String,
        val create: () -> BluetoothSocket?
    )
}
