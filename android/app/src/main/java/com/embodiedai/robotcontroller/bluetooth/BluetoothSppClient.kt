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
import androidx.core.content.ContextCompat
import java.io.IOException
import java.util.UUID
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

data class BondedDeviceInfo(
    val name: String,
    val address: String
)

class BluetoothSppClient(private val context: Context) {
    private val adapter: BluetoothAdapter? =
        context.getSystemService(BluetoothManager::class.java)?.adapter
    private val mainHandler = Handler(Looper.getMainLooper())
    private val ioExecutor = Executors.newSingleThreadExecutor()
    private val readExecutor = Executors.newSingleThreadExecutor()
    private val stateLock = Any()
    private val sppUuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

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
    fun bondedDevices(): List<BondedDeviceInfo> {
        if (!hasConnectPermission()) {
            return emptyList()
        }

        return adapter?.bondedDevices
            ?.map { device ->
                BondedDeviceInfo(
                    name = device.name ?: "Unknown Device",
                    address = device.address
                )
            }
            ?.sortedBy { it.name }
            .orEmpty()
    }

    @SuppressLint("MissingPermission")
    fun connect(address: String, onResult: (Boolean, String) -> Unit) {
        if (!hasConnectPermission()) {
            postResult(onResult, false, "Bluetooth permission is not granted")
            return
        }

        val localAdapter = adapter
        if (localAdapter == null || !localAdapter.isEnabled) {
            postResult(onResult, false, "Bluetooth is disabled")
            return
        }

        val token = nextConnectionToken()
        closeSocket()

        ioExecutor.execute {
            try {
                val bonded = localAdapter.bondedDevices?.any { it.address == address } == true
                if (!bonded) {
                    postResult(onResult, false, "Pair device in Android Bluetooth settings first")
                    return@execute
                }
                val device = localAdapter.getRemoteDevice(address)
                val newSocket = device.createRfcommSocketToServiceRecord(sppUuid)
                newSocket.connect()
                if (!setConnectedSocket(newSocket, token)) {
                    closeQuietly(newSocket)
                    return@execute
                }
                startReadPump(newSocket, token)
                postResult(onResult, true, "Connected to ${device.displayName()}")
            } catch (error: IOException) {
                closeSocket()
                postResult(onResult, false, error.message ?: "Connection failed")
            } catch (error: IllegalArgumentException) {
                closeSocket()
                postResult(onResult, false, error.message ?: "Invalid Bluetooth address")
            }
        }
    }

    fun disconnect(stopFrame: ByteArray? = null, onDone: (() -> Unit)? = null) {
        nextConnectionToken()
        val targetSocket = socket
        val completed = AtomicBoolean(false)

        fun finishDisconnect() {
            if (completed.compareAndSet(false, true)) {
                closeSocketIfCurrent(targetSocket)
                if (onDone != null) {
                    mainHandler.post(onDone)
                }
            }
        }

        if ((targetSocket == null) || (stopFrame == null)) {
            finishDisconnect()
            return
        }

        ioExecutor.execute {
            try {
                targetSocket.outputStream.write(stopFrame)
                targetSocket.outputStream.flush()
            } catch (_: IOException) {
            } finally {
                finishDisconnect()
            }
        }

        mainHandler.postDelayed(::finishDisconnect, DISCONNECT_SEND_GRACE_MS)
    }

    fun send(frame: ByteArray): Boolean {
        val currentSocket = socket ?: return false
        return try {
            currentSocket.outputStream.write(frame)
            currentSocket.outputStream.flush()
            true
        } catch (_: IOException) {
            closeSocketIfCurrent(currentSocket)
            false
        }
    }

    fun sendAsync(frame: ByteArray, onFailure: (() -> Unit)? = null) {
        ioExecutor.execute {
            if (!send(frame) && onFailure != null) {
                mainHandler.post(onFailure)
            }
        }
    }

    private fun nextConnectionToken(): Int {
        synchronized(stateLock) {
            connectionToken += 1
            return connectionToken
        }
    }

    private fun setConnectedSocket(newSocket: BluetoothSocket, token: Int): Boolean {
        synchronized(stateLock) {
            if (token != connectionToken) {
                return false
            }
            socket = newSocket
            return true
        }
    }

    private fun startReadPump(activeSocket: BluetoothSocket, token: Int) {
        readExecutor.execute {
            val buffer = ByteArray(BLUETOOTH_READ_BUF_SIZE)

            try {
                while (isCurrentSocket(activeSocket, token)) {
                    if (activeSocket.inputStream.read(buffer) < 0) {
                        break
                    }
                }
            } catch (_: IOException) {
            } finally {
                closeSocketIfCurrent(activeSocket)
            }
        }
    }

    private fun isCurrentSocket(activeSocket: BluetoothSocket, token: Int): Boolean {
        synchronized(stateLock) {
            return (socket === activeSocket) && (connectionToken == token)
        }
    }

    private fun closeSocket() {
        val socketToClose: BluetoothSocket?
        synchronized(stateLock) {
            socketToClose = socket
            socket = null
        }
        closeQuietly(socketToClose)
    }

    private fun closeSocketIfCurrent(socketToClose: BluetoothSocket?) {
        if (socketToClose == null) {
            return
        }
        synchronized(stateLock) {
            if (socket !== socketToClose) {
                return
            }
            socket = null
        }
        closeQuietly(socketToClose)
    }

    private fun closeQuietly(socketToClose: BluetoothSocket?) {
        try {
            socketToClose?.close()
        } catch (_: IOException) {
        }
    }

    private fun postResult(onResult: (Boolean, String) -> Unit, success: Boolean, message: String) {
        mainHandler.post {
            onResult(success, message)
        }
    }

    @SuppressLint("MissingPermission")
    private fun BluetoothDevice.displayName(): String {
        return name ?: address
    }

    private companion object {
        const val BLUETOOTH_READ_BUF_SIZE = 256
        const val DISCONNECT_SEND_GRACE_MS = 150L
    }
}
