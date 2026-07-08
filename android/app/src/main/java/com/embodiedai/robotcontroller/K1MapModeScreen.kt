package com.embodiedai.robotcontroller

import android.graphics.Bitmap
import android.graphics.Paint
import android.os.SystemClock
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.DrawTransform
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.withTransform
import androidx.compose.ui.graphics.nativeCanvas
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.onSizeChanged
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.zIndex
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner
import com.embodiedai.robotcontroller.bluetooth.BondedDeviceInfo
import com.embodiedai.robotcontroller.bluetooth.K1MapBluetoothClient
import com.embodiedai.robotcontroller.protocol.K1Frame
import com.embodiedai.robotcontroller.protocol.K1AutoMapRoomSize
import com.embodiedai.robotcontroller.protocol.K1BridgeCommand
import com.embodiedai.robotcontroller.protocol.K1BridgeState
import com.embodiedai.robotcontroller.protocol.K1BridgeStatus
import com.embodiedai.robotcontroller.protocol.K1MapCommand
import com.embodiedai.robotcontroller.protocol.K1MappingMode
import com.embodiedai.robotcontroller.protocol.K1MobileProtocol
import com.embodiedai.robotcontroller.protocol.K1MapInfo
import com.embodiedai.robotcontroller.protocol.K1NavPath
import com.embodiedai.robotcontroller.protocol.K1MapState
import com.embodiedai.robotcontroller.protocol.K1MapControlStatus
import com.embodiedai.robotcontroller.protocol.K1MapLibraryCommand
import com.embodiedai.robotcontroller.protocol.K1MapLibraryEntry
import com.embodiedai.robotcontroller.protocol.K1MessageType
import com.embodiedai.robotcontroller.protocol.K1RobotPose
import com.embodiedai.robotcontroller.protocol.decodeMapLibraryList
import com.embodiedai.robotcontroller.protocol.decodeMapLibraryStatus
import com.embodiedai.robotcontroller.protocol.decodeMapRegionsData
import com.embodiedai.robotcontroller.protocol.decodeBridgeStatus
import com.embodiedai.robotcontroller.protocol.decodeHelloName
import com.embodiedai.robotcontroller.protocol.decodeMapInfo
import com.embodiedai.robotcontroller.protocol.decodeMapTile
import com.embodiedai.robotcontroller.protocol.decodeMapControlStatus
import com.embodiedai.robotcontroller.protocol.decodeNavPath
import com.embodiedai.robotcontroller.protocol.decodeRobotPose
import com.embodiedai.robotcontroller.regions.DEFAULT_REGION_COLORS
import com.embodiedai.robotcontroller.regions.MapRegion
import com.embodiedai.robotcontroller.regions.OccupancySnapshot
import com.embodiedai.robotcontroller.regions.RegionPoint
import com.embodiedai.robotcontroller.regions.buildRegionDocument
import com.embodiedai.robotcontroller.regions.closeable
import com.embodiedai.robotcontroller.regions.parseRegionDocument
import com.embodiedai.robotcontroller.regions.pointInPolygon
import com.embodiedai.robotcontroller.regions.polygonCentroid
import com.embodiedai.robotcontroller.regions.toJsonString
import kotlinx.coroutines.launch
import kotlinx.coroutines.delay
import java.time.LocalTime
import java.time.format.DateTimeFormatter
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sin

private enum class K1EndpointKind {
    UNKNOWN,
    SUPERVISOR,
    BRIDGE
}

private enum class K1ConnectionPhase {
    DISCONNECTED,
    SOCKET_CONNECTING,
    WAIT_HELLO,
    SUPERVISOR,
    BRIDGE,
    RECOVERING,
    DISCONNECTING
}

private enum class K1BridgeButtonMode {
    DISABLED,
    START,
    STARTING,
    STOP,
    STOPPING,
    CLEANING,
    CONNECTING,
    DETECTING
}

private enum class K1MapButtonMode {
    DISABLED,
    START,
    STARTING,
    STOP,
    STOPPING,
    SAVING
}

private enum class K1MapWorkMode {
    DRIVE,
    EDIT
}

private data class K1ActionButtonMode(
    val text: String,
    val color: Color,
    val clickEnabled: Boolean
)

private fun preferredK1MapDevice(devices: List<BondedDeviceInfo>): BondedDeviceInfo? {
    return devices.firstOrNull { it.name.equals("bianbu", ignoreCase = true) }
        ?: devices.firstOrNull {
            val name = it.name.lowercase()
            name.contains("k1") || name.contains("muse") || name.contains("bianbu")
        }
        ?: devices.firstOrNull()
}

private fun bridgeActionMode(
    connected: Boolean,
    endpointKind: K1EndpointKind,
    bridgeState: K1BridgeState,
    mapState: K1MapState,
    bridgeBusy: Boolean,
    mapBusy: Boolean,
    connecting: Boolean,
    cleanupStarted: Boolean,
    reconnectAfterBridgeStart: Boolean
): K1ActionButtonMode {
    val onlineBridge = connected && endpointKind == K1EndpointKind.BRIDGE
    val supervisor = connected && endpointKind == K1EndpointKind.SUPERVISOR
    val mode = when {
        cleanupStarted -> K1BridgeButtonMode.CLEANING
        reconnectAfterBridgeStart -> K1BridgeButtonMode.STARTING
        supervisor && (bridgeBusy || bridgeState == K1BridgeState.STARTING) -> K1BridgeButtonMode.STARTING
        supervisor -> K1BridgeButtonMode.START
        onlineBridge && (bridgeBusy || bridgeState == K1BridgeState.STOPPING) -> K1BridgeButtonMode.STOPPING
        onlineBridge -> K1BridgeButtonMode.STOP
        connecting -> K1BridgeButtonMode.CONNECTING
        connected -> K1BridgeButtonMode.DETECTING
        else -> K1BridgeButtonMode.DISABLED
    }
    val clickEnabled = when (mode) {
        K1BridgeButtonMode.START -> supervisor && !bridgeBusy && !mapBusy && !connecting && !cleanupStarted
        K1BridgeButtonMode.STOP -> onlineBridge && !bridgeBusy && !mapBusy && !cleanupStarted && mapState == K1MapState.IDLE
        else -> false
    }
    val color = when (mode) {
        K1BridgeButtonMode.START,
        K1BridgeButtonMode.STARTING -> Color(0xFF20A66B)
        K1BridgeButtonMode.STOP,
        K1BridgeButtonMode.STOPPING,
        K1BridgeButtonMode.CLEANING -> Color(0xFF7D5FFF)
        K1BridgeButtonMode.CONNECTING,
        K1BridgeButtonMode.DETECTING,
        K1BridgeButtonMode.DISABLED -> Color(0xFF3A414D)
    }
    val text = when (mode) {
        K1BridgeButtonMode.START -> "Start Bridge"
        K1BridgeButtonMode.STARTING -> "Starting Bridge"
        K1BridgeButtonMode.STOP -> "Stop Bridge"
        K1BridgeButtonMode.STOPPING -> "Stopping Bridge"
        K1BridgeButtonMode.CLEANING -> "Cleaning"
        K1BridgeButtonMode.CONNECTING -> "Connecting"
        K1BridgeButtonMode.DETECTING -> "Detecting"
        K1BridgeButtonMode.DISABLED -> "Bridge"
    }
    return K1ActionButtonMode(text, color, clickEnabled)
}

private fun mapActionMode(
    onlineBridge: Boolean,
    mapState: K1MapState,
    bridgeBusy: Boolean,
    mapBusy: Boolean,
    connecting: Boolean,
    cleanupStarted: Boolean,
    pendingMapCommand: K1MapCommand?,
    activeMapCommand: K1MapCommand?
): K1ActionButtonMode {
    val activeCommand = pendingMapCommand ?: activeMapCommand
    val mode = when {
        !onlineBridge -> K1MapButtonMode.DISABLED
        activeCommand == K1MapCommand.START_MAPPING || mapState == K1MapState.STARTING -> K1MapButtonMode.STARTING
        activeCommand == K1MapCommand.STOP_MAPPING || mapState == K1MapState.STOPPING -> K1MapButtonMode.STOPPING
        activeCommand == K1MapCommand.SAVE_MAP_MANUAL || mapState == K1MapState.SAVING -> K1MapButtonMode.SAVING
        mapState == K1MapState.MAPPING -> K1MapButtonMode.STOP
        else -> K1MapButtonMode.START
    }
    val transitionActive = bridgeBusy || mapBusy || connecting || cleanupStarted
    val clickEnabled = when (mode) {
        K1MapButtonMode.START -> onlineBridge && !transitionActive &&
            (mapState == K1MapState.IDLE || mapState == K1MapState.ERROR)
        K1MapButtonMode.STOP -> onlineBridge && !transitionActive && mapState == K1MapState.MAPPING
        else -> false
    }
    val color = when (mode) {
        K1MapButtonMode.START,
        K1MapButtonMode.STARTING -> Color(0xFF4F8CFF)
        K1MapButtonMode.STOP,
        K1MapButtonMode.STOPPING -> Color(0xFFE53935)
        K1MapButtonMode.SAVING -> Color(0xFFE6A23A)
        K1MapButtonMode.DISABLED -> Color(0xFF3A414D)
    }
    val text = when (mode) {
        K1MapButtonMode.START -> "Start Map"
        K1MapButtonMode.STARTING -> "Starting Map"
        K1MapButtonMode.STOP -> "Stop Map"
        K1MapButtonMode.STOPPING -> "Stopping Map"
        K1MapButtonMode.SAVING -> "Saving Map"
        K1MapButtonMode.DISABLED -> "Start Map"
    }
    return K1ActionButtonMode(text, color, clickEnabled)
}

private class MobileMapBuffer {
    var info by mutableStateOf<K1MapInfo?>(null)
        private set
    var bitmap by mutableStateOf<Bitmap?>(null)
        private set
    var pose by mutableStateOf<K1RobotPose?>(null)
        private set
    var navPath by mutableStateOf<K1NavPath?>(null)
        private set
    var occupancy by mutableStateOf<ByteArray?>(null)
        private set
    var isLastMap by mutableStateOf(false)
        private set
    var imageVersion by mutableIntStateOf(0)
        private set
    var status by mutableStateOf("Waiting for K1 map")
        private set
    private var lastImageRefreshMs = 0L

    fun applyFrame(frame: K1Frame) {
        when (frame.type) {
            K1MessageType.MAP_INFO -> {
                val decoded = frame.decodeMapInfo() ?: return
                val previousInfo = info
                val previousBitmap = bitmap
                isLastMap = false
                if (previousInfo != null &&
                    previousBitmap != null &&
                    sameMapCanvas(previousInfo, decoded)
                ) {
                    info = decoded
                    status = "Map ${decoded.width}x${decoded.height} @ ${decoded.resolution} m"
                    return
                }
                val nextBitmap = Bitmap.createBitmap(decoded.width, decoded.height, Bitmap.Config.ARGB_8888)
                nextBitmap.eraseColor(UNKNOWN_COLOR)
                occupancy = ByteArray(decoded.width * decoded.height) { (-1).toByte() }
                if (previousInfo != null && previousBitmap != null) {
                    carryOverPreviousMap(previousBitmap, previousInfo, nextBitmap, decoded)
                }
                info = decoded
                bitmap = nextBitmap
                status = "Map ${decoded.width}x${decoded.height} @ ${decoded.resolution} m"
                refreshImage(force = true)
            }
            K1MessageType.MAP_TILE -> {
                val decoded = frame.decodeMapTile() ?: return
                val current = info
                val target = bitmap
                if (current == null || target == null) {
                    status = "drop tile without map info"
                    return
                }
                if (decoded.mapVersion != current.mapVersion) {
                    status = "drop stale tile v${decoded.mapVersion}; current v${current.mapVersion}"
                    return
                }
                if (!tileInBounds(current, decoded)) {
                    status = "drop out-of-bounds tile ${decoded.x},${decoded.y} ${decoded.width}x${decoded.height}"
                    return
                }
                applyTile(target, current, decoded)
                refreshImage(force = false)
            }
            K1MessageType.ROBOT_POSE -> {
                pose = frame.decodeRobotPose()
            }
            K1MessageType.NAV_PATH -> {
                navPath = frame.decodeNavPath()
            }
            K1MessageType.HELLO -> {
                status = "K1 bridge connected"
            }
            K1MessageType.ERROR -> {
                status = "K1 bridge error"
            }
            else -> Unit
        }
    }

    fun markLiveMap() {
        isLastMap = false
    }

    fun markLastMap() {
        if (bitmap != null && info != null) {
            isLastMap = true
        }
    }

    private fun applyTile(target: Bitmap, current: K1MapInfo, decoded: com.embodiedai.robotcontroller.protocol.K1MapTile) {
        val pixels = IntArray(decoded.width * decoded.height)
        val grid = occupancy
        for (row in 0 until decoded.height) {
            for (col in 0 until decoded.width) {
                val sourceIndex = row * decoded.width + col
                val destRow = decoded.height - 1 - row
                pixels[destRow * decoded.width + col] =
                    occupancyColor(decoded.raw[sourceIndex].toInt())
                grid?.set((decoded.y + row) * current.width + decoded.x + col, decoded.raw[sourceIndex])
            }
        }
        val bitmapY = current.height - decoded.y - decoded.height
        target.setPixels(
            pixels,
            0,
            decoded.width,
            decoded.x,
            bitmapY,
            decoded.width,
            decoded.height
        )
    }

    private fun sameMapCanvas(previous: K1MapInfo, next: K1MapInfo): Boolean {
        return previous.mapVersion == next.mapVersion &&
            previous.width == next.width &&
            previous.height == next.height &&
            abs(previous.resolution - next.resolution) < 0.000001f &&
            abs(previous.originX - next.originX) < 0.000001f &&
            abs(previous.originY - next.originY) < 0.000001f &&
            abs(previous.originYaw - next.originYaw) < 0.000001f
    }

    private fun carryOverPreviousMap(
        previousBitmap: Bitmap,
        previousInfo: K1MapInfo,
        nextBitmap: Bitmap,
        nextInfo: K1MapInfo
    ) {
        if (abs(previousInfo.resolution - nextInfo.resolution) > 0.000001f ||
            abs(previousInfo.originYaw - nextInfo.originYaw) > 0.000001f
        ) {
            return
        }

        val dx = ((previousInfo.originX - nextInfo.originX) / nextInfo.resolution).roundToInt()
        val dy = ((previousInfo.originY - nextInfo.originY) / nextInfo.resolution).roundToInt()
        val destX = dx
        val destY = nextInfo.height - previousInfo.height - dy

        val copyLeft = max(0, destX)
        val copyTop = max(0, destY)
        val copyRight = min(nextInfo.width, destX + previousInfo.width)
        val copyBottom = min(nextInfo.height, destY + previousInfo.height)
        val copyWidth = copyRight - copyLeft
        val copyHeight = copyBottom - copyTop
        if (copyWidth <= 0 || copyHeight <= 0) {
            return
        }

        val sourceX = copyLeft - destX
        val sourceY = copyTop - destY
        val pixels = IntArray(copyWidth * copyHeight)
        previousBitmap.getPixels(
            pixels,
            0,
            copyWidth,
            sourceX,
            sourceY,
            copyWidth,
            copyHeight
        )
        nextBitmap.setPixels(
            pixels,
            0,
            copyWidth,
            copyLeft,
            copyTop,
            copyWidth,
            copyHeight
        )
    }

    private fun refreshImage(force: Boolean) {
        val now = SystemClock.uptimeMillis()
        if (force || now - lastImageRefreshMs >= IMAGE_REFRESH_MS) {
            imageVersion += 1
            lastImageRefreshMs = now
        }
    }

    private fun tileInBounds(info: K1MapInfo, tile: com.embodiedai.robotcontroller.protocol.K1MapTile): Boolean {
        return tile.width > 0 &&
            tile.height > 0 &&
            tile.x >= 0 &&
            tile.y >= 0 &&
            tile.x + tile.width <= info.width &&
            tile.y + tile.height <= info.height
    }

    private fun occupancyColor(value: Int): Int {
        return when {
            value < 0 -> UNKNOWN_COLOR
            value == 0 -> FREE_COLOR
            value >= 100 -> OCCUPIED_COLOR
            else -> {
                val level = 238 - (value * 160 / 100)
                android.graphics.Color.rgb(level, level, level)
            }
        }
    }

    fun snapshot(): OccupancySnapshot? {
        val current = info ?: return null
        val raw = occupancy ?: return null
        if (raw.size != current.width * current.height) {
            return null
        }
        return OccupancySnapshot(
            width = current.width,
            height = current.height,
            resolution = current.resolution,
            originX = current.originX,
            originY = current.originY,
            originYaw = current.originYaw,
            data = raw.copyOf()
        )
    }

    private companion object {
        val UNKNOWN_COLOR: Int = 0xFF707782.toInt()
        val FREE_COLOR: Int = 0xFFE8EEF5.toInt()
        val OCCUPIED_COLOR: Int = 0xFF111820.toInt()
        const val IMAGE_REFRESH_MS = 150L
    }
}

@Composable
fun K1MapModeScreen(
    bluetoothClient: K1MapBluetoothClient,
    hasBluetoothPermission: Boolean,
    devices: List<BondedDeviceInfo>,
    onRequestPermission: () -> Unit,
    onRefreshDevices: () -> Unit,
    onBack: () -> Unit
) {
    val initialDevice = preferredK1MapDevice(devices)
    var selectedAddress by remember { mutableStateOf<String?>(initialDevice?.address) }
    var selectedName by remember { mutableStateOf(initialDevice?.name ?: "No device") }
    var connected by remember { mutableStateOf(false) }
    var connecting by remember { mutableStateOf(false) }
    var connectionPhase by remember { mutableStateOf(K1ConnectionPhase.DISCONNECTED) }
    var endpointKind by remember { mutableStateOf(K1EndpointKind.UNKNOWN) }
    var bridgeState by remember { mutableStateOf(K1BridgeState.ERROR) }
    var mapState by remember { mutableStateOf(K1MapState.IDLE) }
    var bridgeBusy by remember { mutableStateOf(false) }
    var mapBusy by remember { mutableStateOf(false) }
    var lastMapBase by remember { mutableStateOf("") }
    var reconnectAfterBridgeStart by remember { mutableStateOf(false) }
    var autoReconnectToken by remember { mutableIntStateOf(0) }
    var connectAttemptToken by remember { mutableIntStateOf(0) }
    var cleanupStarted by remember { mutableStateOf(false) }
    var lastBridgeMonitor by remember { mutableStateOf("") }
    var lastMapMonitor by remember { mutableStateOf("") }
    var pendingMapCommand by remember { mutableStateOf<K1MapCommand?>(null) }
    var activeMapCommand by remember { mutableStateOf<K1MapCommand?>(null) }
    var autoMappingActive by remember { mutableStateOf(false) }
    var mapStartModeDialogVisible by remember { mutableStateOf(false) }
    var autoRoomDialogVisible by remember { mutableStateOf(false) }
    var autoCustomDialogVisible by remember { mutableStateOf(false) }
    var selectedAutoRoomSize by remember { mutableStateOf(K1AutoMapRoomSize.MEDIUM) }
    var customAutoXText by remember { mutableStateOf("10") }
    var customAutoYText by remember { mutableStateOf("10") }
    var pendingMapSentAt by remember { mutableStateOf(0L) }
    var pendingMapRetryCount by remember { mutableIntStateOf(0) }
    var logExpanded by remember { mutableStateOf(false) }
    val logs = remember { mutableStateListOf("Select paired K1 and link") }
    var manualEnabled by remember { mutableStateOf(false) }
    var speedScale by remember { mutableFloatStateOf(0.25f) }
    var omegaScale by remember { mutableFloatStateOf(0.25f) }
    var vx by remember { mutableFloatStateOf(0f) }
    var vy by remember { mutableFloatStateOf(0f) }
    var omega by remember { mutableFloatStateOf(0f) }
    var txSeq by remember { mutableStateOf(1L) }
    var resetViewToken by remember { mutableIntStateOf(0) }
    var mapRotationTurns by remember { mutableIntStateOf(0) }
    val mapBuffer = remember { MobileMapBuffer() }
    var workMode by remember { mutableStateOf(K1MapWorkMode.DRIVE) }
    val libraryMaps = remember { mutableStateListOf<K1MapLibraryEntry>() }
    var selectedLibraryMap by remember { mutableStateOf<K1MapLibraryEntry?>(null) }
    val regions = remember { mutableStateListOf<MapRegion>() }
    var selectedRegionId by remember { mutableStateOf<String?>(null) }
    var draftVertices by remember { mutableStateOf<List<RegionPoint>>(emptyList()) }
    var redrawRegionId by remember { mutableStateOf<String?>(null) }
    var nameDialogVisible by remember { mutableStateOf(false) }
    var nameDialogText by remember { mutableStateOf("") }
    var leaveEditConfirmVisible by remember { mutableStateOf(false) }
    var deleteConfirmVisible by remember { mutableStateOf(false) }
    var pendingDraftName by remember { mutableStateOf(false) }
    var libraryBusy by remember { mutableStateOf(false) }
    var regionStatus by remember { mutableStateOf("No map selected") }
    var editBaselineRegions by remember { mutableStateOf<List<MapRegion>>(emptyList()) }
    var editBaselineMap by remember { mutableStateOf<K1MapLibraryEntry?>(null) }
    val latestConnected by rememberUpdatedState(connected)
    val latestManualEnabled by rememberUpdatedState(manualEnabled)
    val latestAutoMappingActive by rememberUpdatedState(autoMappingActive)
    val latestEndpointKind by rememberUpdatedState(endpointKind)
    val latestVx by rememberUpdatedState(vx)
    val latestVy by rememberUpdatedState(vy)
    val latestOmega by rememberUpdatedState(omega)
    val scope = rememberCoroutineScope()
    val lifecycleOwner = LocalLifecycleOwner.current

    fun nextSeq(): Long {
        txSeq = (txSeq + 1L) and 0xffffffffL
        return txSeq
    }

    fun addLog(message: String) {
        val time = LocalTime.now().format(DateTimeFormatter.ofPattern("HH:mm:ss"))
        logs.add(0, "[$time] $message")
        while (logs.size > 50) {
            logs.removeAt(logs.lastIndex)
        }
    }

    fun updateAutoMappingFromStatus(status: K1MapControlStatus) {
        val message = status.message.lowercase()
        if (status.state == K1MapState.IDLE || status.state == K1MapState.ERROR) {
            if (status.command == K1MapCommand.STOP_MAPPING ||
                status.command == K1MapCommand.QUERY_MAPPING ||
                status.command == K1MapCommand.START_MAPPING
            ) {
                autoMappingActive = false
            }
        }
        if (status.state == K1MapState.MAPPING || status.state == K1MapState.STARTING) {
            if (message.contains("auto mapping")) {
                autoMappingActive = true
            } else if (message.contains("manual mapping")) {
                autoMappingActive = false
            }
        }
    }

    fun resetTeleopInputs() {
        vx = 0f
        vy = 0f
        omega = 0f
    }

    fun sendK1Stop(reason: String) {
        resetTeleopInputs()
        if (connected && endpointKind == K1EndpointKind.BRIDGE) {
            bluetoothClient.sendAsync(K1MobileProtocol.stop(nextSeq())) { message ->
                addLog(message)
            }
        }
        addLog(reason)
    }

    fun resetDisconnectedState() {
        connected = false
        connecting = false
        connectionPhase = K1ConnectionPhase.DISCONNECTED
        manualEnabled = false
        endpointKind = K1EndpointKind.UNKNOWN
        bridgeBusy = false
        mapBusy = false
        autoMappingActive = false
        cleanupStarted = false
        reconnectAfterBridgeStart = false
        resetTeleopInputs()
    }

    fun cleanupAndDisconnect(reason: String, afterCleanup: (() -> Unit)? = null) {
        if (cleanupStarted) {
            addLog("Cleanup already running")
            return
        }
        cleanupStarted = true
        connectionPhase = K1ConnectionPhase.DISCONNECTING
        manualEnabled = false
        resetTeleopInputs()
        addLog(reason)
        if (connected && endpointKind == K1EndpointKind.BRIDGE) {
            bridgeBusy = true
            bridgeState = K1BridgeState.STOPPING
            bluetoothClient.sendAsync(K1MobileProtocol.stop(nextSeq())) { message ->
                addLog(message)
            }
            bluetoothClient.sendAsync(
                K1MobileProtocol.bridgeControl(nextSeq(), K1BridgeCommand.STOP_BRIDGE)
            ) { message ->
                addLog(message)
            }
            scope.launch {
                delay(1200L)
                bluetoothClient.disconnect {
                    resetDisconnectedState()
                    addLog("Disconnected")
                    afterCleanup?.invoke()
                }
            }
        } else {
            bluetoothClient.disconnect {
                resetDisconnectedState()
                addLog("Disconnected")
                afterCleanup?.invoke()
            }
        }
    }

    fun connectSelected() {
        if (connecting) {
            addLog("Already connecting")
            return
        }
        val address = selectedAddress
        if (address == null) {
            addLog("Select paired K1 first")
            return
        }
        connecting = true
        connectionPhase = K1ConnectionPhase.SOCKET_CONNECTING
        connectAttemptToken += 1
        endpointKind = K1EndpointKind.UNKNOWN
        bridgeState = K1BridgeState.ERROR
        addLog("Connecting to $selectedName")
        bluetoothClient.connect(
            address = address,
            onFrame = { frame ->
                when (frame.type) {
                    K1MessageType.HELLO -> {
                        val name = frame.decodeHelloName().orEmpty()
                        endpointKind = if (name.contains("supervisor")) {
                            connectionPhase = K1ConnectionPhase.SUPERVISOR
                            if (reconnectAfterBridgeStart) {
                                bridgeState = K1BridgeState.STARTING
                                bridgeBusy = true
                                addLog("K1 supervisor still active; waiting for bridge")
                                scope.launch {
                                    delay(1000L)
                                    if (reconnectAfterBridgeStart &&
                                        connected &&
                                        endpointKind == K1EndpointKind.SUPERVISOR &&
                                        !cleanupStarted
                                    ) {
                                        bluetoothClient.disconnect {
                                            connected = false
                                            connecting = false
                                            connectionPhase = K1ConnectionPhase.RECOVERING
                                            endpointKind = K1EndpointKind.UNKNOWN
                                            bridgeBusy = true
                                            autoReconnectToken += 1
                                        }
                                    }
                                }
                            } else {
                                bridgeState = K1BridgeState.SUPERVISOR
                                bridgeBusy = false
                                addLog("Connected to K1 supervisor")
                            }
                            K1EndpointKind.SUPERVISOR
                        } else {
                            connectionPhase = K1ConnectionPhase.BRIDGE
                            bridgeState = K1BridgeState.ONLINE
                            bridgeBusy = false
                            reconnectAfterBridgeStart = false
                            addLog("Connected to K1 mobile bridge")
                            bluetoothClient.sendAsync(
                                K1MobileProtocol.mapControl(nextSeq(), K1MapCommand.QUERY_MAPPING)
                            ) { message ->
                                addLog(message)
                            }
                            bluetoothClient.sendAsync(
                                K1MobileProtocol.mapLibraryRequest(nextSeq(), K1MapLibraryCommand.LIST_MAPS)
                            ) { message ->
                                addLog(message)
                            }
                            K1EndpointKind.BRIDGE
                        }
                        mapBuffer.applyFrame(frame)
                    }
                    K1MessageType.BRIDGE_STATUS -> {
                        frame.decodeBridgeStatus()?.let { status ->
                            if (status.command == K1BridgeCommand.QUERY_BRIDGE &&
                                (bridgeBusy || cleanupStarted || connectionPhase == K1ConnectionPhase.RECOVERING)
                            ) {
                                val key = "${status.state}:${status.command}:${status.success}:${status.message}"
                                if (key != lastBridgeMonitor) {
                                    addLog("Bridge ${status.state.name}: ${status.message}")
                                }
                                lastBridgeMonitor = key
                                return@let
                            }
                            bridgeState = status.state
                            bridgeBusy = status.state == K1BridgeState.STARTING ||
                                status.state == K1BridgeState.STOPPING
                            val key = "${status.state}:${status.command}:${status.success}:${status.message}"
                            if (status.command != K1BridgeCommand.QUERY_BRIDGE || key != lastBridgeMonitor) {
                                addLog("Bridge ${status.state.name}: ${status.message}")
                            }
                            lastBridgeMonitor = key
                            if (status.command == K1BridgeCommand.START_BRIDGE &&
                                status.success &&
                                status.state == K1BridgeState.STARTING &&
                                !reconnectAfterBridgeStart
                            ) {
                                reconnectAfterBridgeStart = true
                                connectionPhase = K1ConnectionPhase.RECOVERING
                            }
                        }
                    }
                    K1MessageType.MAP_CONTROL_STATUS -> {
                        frame.decodeMapControlStatus()?.let { status ->
                            updateAutoMappingFromStatus(status)
                            val inFlightMapCommand = pendingMapCommand ?: activeMapCommand
                            if (status.command == K1MapCommand.START_MAPPING &&
                                status.state == K1MapState.STARTING &&
                                mapState == K1MapState.MAPPING &&
                                inFlightMapCommand == null
                            ) {
                                val key = "${status.state}:${status.command}:${status.success}:${status.message}:${status.mapBase}"
                                if (key != lastMapMonitor) {
                                    addLog("Map ${status.state.name}: ${status.message}")
                                }
                                lastMapMonitor = key
                                return@let
                            }
                            if (status.command != K1MapCommand.QUERY_MAPPING &&
                                status.state == K1MapState.ERROR &&
                                status.message.contains("another mapping command", ignoreCase = true) &&
                                inFlightMapCommand == status.command
                            ) {
                                mapBusy = true
                                mapState = when (status.command) {
                                    K1MapCommand.START_MAPPING -> K1MapState.STARTING
                                    K1MapCommand.SAVE_MAP_MANUAL -> K1MapState.SAVING
                                    K1MapCommand.STOP_MAPPING -> K1MapState.STOPPING
                                    K1MapCommand.QUERY_MAPPING -> mapState
                                }
                                val key = "${status.state}:${status.command}:${status.success}:${status.message}:${status.mapBase}"
                                if (key != lastMapMonitor) {
                                    addLog("Map ${status.state.name}: ${status.message}")
                                }
                                lastMapMonitor = key
                                return@let
                            }
                            if (status.command != K1MapCommand.QUERY_MAPPING &&
                                pendingMapCommand == status.command
                            ) {
                                pendingMapCommand = null
                                pendingMapRetryCount = 0
                            }
                            if (status.command == K1MapCommand.QUERY_MAPPING && mapBusy) {
                                if (pendingMapCommand == null &&
                                    (status.state == K1MapState.MAPPING ||
                                        status.state == K1MapState.IDLE ||
                                        status.state == K1MapState.ERROR)
                                ) {
                                    mapState = status.state
                                    mapBusy = false
                                    if (activeMapCommand == K1MapCommand.STOP_MAPPING &&
                                        status.success &&
                                        status.state == K1MapState.IDLE
                                    ) {
                                        mapBuffer.markLastMap()
                                    }
                                    activeMapCommand = null
                                }
                                val key = "${status.state}:${status.command}:${status.success}:${status.message}:${status.mapBase}"
                                if (key != lastMapMonitor) {
                                    addLog("Map ${status.state.name}: ${status.message}")
                                }
                                lastMapMonitor = key
                                return@let
                            }
                            mapState = status.state
                            mapBusy = status.state == K1MapState.STARTING ||
                                status.state == K1MapState.SAVING ||
                                status.state == K1MapState.STOPPING
                            if (status.mapBase.isNotBlank()) {
                                lastMapBase = status.mapBase
                            }
                            if (status.state == K1MapState.MAPPING || status.state == K1MapState.STARTING) {
                                mapBuffer.markLiveMap()
                            }
                            if (status.command == K1MapCommand.STOP_MAPPING && status.success && status.state == K1MapState.IDLE) {
                                mapBuffer.markLastMap()
                            }
                            val key = "${status.state}:${status.command}:${status.success}:${status.message}:${status.mapBase}"
                            if (status.command != K1MapCommand.QUERY_MAPPING || key != lastMapMonitor) {
                                addLog("Map ${status.state.name}: ${status.message}")
                            }
                            lastMapMonitor = key
                            if (status.command != K1MapCommand.QUERY_MAPPING && !mapBusy) {
                                activeMapCommand = null
                            }
                        }
                    }
                    K1MessageType.MAP_INFO,
                    K1MessageType.MAP_TILE -> {
                        mapBuffer.applyFrame(frame)
                        if (endpointKind == K1EndpointKind.BRIDGE &&
                            (pendingMapCommand == K1MapCommand.START_MAPPING ||
                                activeMapCommand == K1MapCommand.START_MAPPING) &&
                            (mapState == K1MapState.STARTING ||
                                mapState == K1MapState.ERROR ||
                                mapState == K1MapState.IDLE)
                        ) {
                            mapState = K1MapState.MAPPING
                            mapBusy = false
                            pendingMapCommand = null
                            activeMapCommand = null
                            pendingMapRetryCount = 0
                            mapBuffer.markLiveMap()
                        }
                    }
                    K1MessageType.MAP_LIBRARY_STATUS -> {
                        frame.decodeMapLibraryStatus()?.let { status ->
                            libraryBusy = false
                            regionStatus = status.message
                            addLog("Library ${status.command.name}: ${status.message}")
                            if (status.command == K1MapLibraryCommand.SAVE_REGIONS && status.success) {
                                selectedLibraryMap = selectedLibraryMap?.copy(hasRegions = true)
                            }
                        }
                    }
                    K1MessageType.MAP_LIBRARY_LIST -> {
                        frame.decodeMapLibraryList()?.let { list ->
                            libraryBusy = false
                            libraryMaps.clear()
                            libraryMaps.addAll(list)
                            if (selectedLibraryMap == null && list.isNotEmpty()) {
                                selectedLibraryMap = list.first()
                            }
                            regionStatus = "${list.size} maps found"
                            addLog(regionStatus)
                        }
                    }
                    K1MessageType.MAP_REGIONS_DATA -> {
                        frame.decodeMapRegionsData()?.let { data ->
                            if (selectedLibraryMap?.yamlName == data.yamlName) {
                                regions.clear()
                                selectedRegionId = null
                                draftVertices = emptyList()
                                redrawRegionId = null
                                if (data.exists) {
                                    val document = parseRegionDocument(data.json)
                                    if (document != null) {
                                        regions.addAll(document.regions)
                                        regionStatus = "${document.regions.size} regions loaded"
                                    } else {
                                        regionStatus = "regions file parse failed"
                                    }
                                } else {
                                    regionStatus = "No regions on board"
                                }
                                addLog(regionStatus)
                            }
                        }
                    }
                    else -> mapBuffer.applyFrame(frame)
                }
            },
            onResult = { success, message ->
                connected = success
                connecting = false
                connectionPhase = if (success) K1ConnectionPhase.WAIT_HELLO else K1ConnectionPhase.DISCONNECTED
                manualEnabled = false
                resetTeleopInputs()
                addLog(message)
            },
            onDisconnected = { message ->
                addLog(message)
                if (cleanupStarted) {
                    resetDisconnectedState()
                } else {
                    connected = false
                    connecting = false
                    connectionPhase = if (reconnectAfterBridgeStart) {
                        K1ConnectionPhase.RECOVERING
                    } else {
                        K1ConnectionPhase.DISCONNECTED
                    }
                    manualEnabled = false
                    resetTeleopInputs()
                    if (reconnectAfterBridgeStart) {
                        bridgeBusy = true
                        endpointKind = K1EndpointKind.UNKNOWN
                        addLog("Bridge is starting; reconnecting")
                        autoReconnectToken += 1
                    } else {
                        endpointKind = K1EndpointKind.UNKNOWN
                        bridgeBusy = false
                    }
                }
            }
        )
    }

    fun sendBridgeCommand(command: K1BridgeCommand) {
        if (cleanupStarted) {
            addLog("Bridge cleanup in progress")
            return
        }
        if (bridgeBusy) {
            addLog("Bridge command in progress")
            return
        }
        bridgeBusy = true
        bridgeState = when (command) {
            K1BridgeCommand.START_BRIDGE -> K1BridgeState.STARTING
            K1BridgeCommand.STOP_BRIDGE -> K1BridgeState.STOPPING
            K1BridgeCommand.QUERY_BRIDGE -> bridgeState
        }
        bluetoothClient.sendAsync(K1MobileProtocol.bridgeControl(nextSeq(), command)) { message ->
            bridgeBusy = false
            addLog(message)
        }
        addLog("Bridge command ${command.name}")
    }

    fun sendMapCommand(
        command: K1MapCommand,
        mode: K1MappingMode = K1MappingMode.MANUAL,
        roomSize: K1AutoMapRoomSize = K1AutoMapRoomSize.MEDIUM,
        customSizeX: Float? = null,
        customSizeY: Float? = null
    ) {
        if (cleanupStarted || bridgeBusy) {
            addLog("K1 state transition in progress")
            return
        }
        if (mapBusy) {
            addLog("Map command in progress")
            return
        }
        mapBusy = true
        mapState = when (command) {
            K1MapCommand.START_MAPPING -> {
                if (mode == K1MappingMode.AUTO) {
                    if (manualEnabled) {
                        manualEnabled = false
                        sendK1Stop("Manual disabled for auto mapping")
                    }
                    autoMappingActive = true
                } else {
                    autoMappingActive = false
                }
                mapBuffer.markLiveMap()
                K1MapState.STARTING
            }
            K1MapCommand.SAVE_MAP_MANUAL -> K1MapState.SAVING
            K1MapCommand.STOP_MAPPING -> K1MapState.STOPPING
            K1MapCommand.QUERY_MAPPING -> mapState
        }
        pendingMapCommand = command
        activeMapCommand = command
        pendingMapSentAt = SystemClock.elapsedRealtime()
        pendingMapRetryCount = 0
        bluetoothClient.sendAsync(
            K1MobileProtocol.mapControl(
                nextSeq(),
                command,
                mode = if (command == K1MapCommand.START_MAPPING) mode else null,
                roomSize = if (command == K1MapCommand.START_MAPPING && mode == K1MappingMode.AUTO) roomSize else null,
                customSizeX = customSizeX,
                customSizeY = customSizeY
            )
        ) { message ->
            mapBusy = false
            pendingMapCommand = null
            activeMapCommand = null
            if (command == K1MapCommand.START_MAPPING) {
                autoMappingActive = false
            }
            addLog(message)
        }
        val modeLabel = if (command == K1MapCommand.START_MAPPING && mode == K1MappingMode.AUTO) {
            if (roomSize == K1AutoMapRoomSize.CUSTOM && customSizeX != null && customSizeY != null) {
                " AUTO ${customSizeX.format2()}x${customSizeY.format2()}m"
            } else {
                " AUTO ${roomSize.label}"
            }
        } else {
            ""
        }
        addLog("Map command ${command.name}$modeLabel")
    }

    fun sendMapLibraryCommand(command: K1MapLibraryCommand, mapName: String = "", regionsJson: String = "") {
        if (!connected || endpointKind != K1EndpointKind.BRIDGE || cleanupStarted) {
            addLog("Map library requires K1 bridge")
            return
        }
        libraryBusy = true
        val frame = try {
            K1MobileProtocol.mapLibraryRequest(nextSeq(), command, mapName, regionsJson)
        } catch (error: IllegalArgumentException) {
            libraryBusy = false
            addLog(error.message ?: "bad map library payload")
            return
        }
        bluetoothClient.sendAsync(frame) { message ->
            libraryBusy = false
            addLog(message)
        }
        addLog("Map library command ${command.name}")
    }

    fun refreshLibraryMaps() {
        sendMapLibraryCommand(K1MapLibraryCommand.LIST_MAPS)
    }

    fun loadLibraryMap(entry: K1MapLibraryEntry) {
        selectedLibraryMap = entry
        regions.clear()
        selectedRegionId = null
        draftVertices = emptyList()
        redrawRegionId = null
        regionStatus = "Loading ${entry.yamlName}"
        sendMapLibraryCommand(K1MapLibraryCommand.LOAD_MAP, entry.yamlName)
        sendMapLibraryCommand(K1MapLibraryCommand.READ_REGIONS, entry.yamlName)
    }

    fun startAddRegion() {
        if (selectedLibraryMap == null || mapBuffer.info == null) {
            addLog("Load a static map first")
            return
        }
        redrawRegionId = null
        selectedRegionId = null
        draftVertices = emptyList()
        pendingDraftName = true
        regionStatus = "Tap map to add region points"
    }

    fun completeDraftRegion() {
        if (!closeable(draftVertices)) {
            addLog("At least 3 points required")
            return
        }
        nameDialogText = if (redrawRegionId != null) {
            regions.firstOrNull { it.id == redrawRegionId }?.name.orEmpty()
        } else {
            "区域 ${regions.size + 1}"
        }
        nameDialogVisible = true
    }

    fun commitDraftRegion(name: String) {
        val trimmed = name.trim()
        if (trimmed.isEmpty() || !closeable(draftVertices)) {
            return
        }
        val targetId = redrawRegionId
        if (targetId != null) {
            val index = regions.indexOfFirst { it.id == targetId }
            if (index >= 0) {
                regions[index] = regions[index].copy(name = trimmed, vertices = draftVertices, center = null)
                selectedRegionId = targetId
            }
        } else {
            val color = DEFAULT_REGION_COLORS[regions.size % DEFAULT_REGION_COLORS.size]
            val region = MapRegion(name = trimmed, color = color, vertices = draftVertices)
            regions.add(region)
            selectedRegionId = region.id
        }
        draftVertices = emptyList()
        redrawRegionId = null
        pendingDraftName = false
        nameDialogVisible = false
        regionStatus = "Region saved locally"
    }

    fun renameSelectedRegion() {
        val region = regions.firstOrNull { it.id == selectedRegionId } ?: return
        draftVertices = emptyList()
        redrawRegionId = null
        pendingDraftName = false
        nameDialogText = region.name
        nameDialogVisible = true
    }

    fun deleteSelectedRegion() {
        val target = selectedRegionId ?: return
        regions.removeAll { it.id == target }
        selectedRegionId = null
        draftVertices = emptyList()
        redrawRegionId = null
        regionStatus = "Region deleted locally"
    }

    fun redrawSelectedRegion() {
        val target = selectedRegionId ?: return
        val region = regions.firstOrNull { it.id == target } ?: return
        redrawRegionId = target
        draftVertices = region.vertices
        pendingDraftName = true
        regionStatus = "Tap map to redraw selected region"
    }

    fun trySaveRegionsToBoard(): Boolean {
        val entry = selectedLibraryMap
        val snapshot = mapBuffer.snapshot()
        if (entry == null || snapshot == null) {
            addLog("Load a static map first")
            return false
        }
        val document = buildRegionDocument(entry.yamlName, entry.imageName, snapshot, regions)
        if (document == null) {
            regionStatus = "A region has no free center"
            addLog(regionStatus)
            return false
        }
        sendMapLibraryCommand(K1MapLibraryCommand.SAVE_REGIONS, entry.yamlName, document.toJsonString())
        return true
    }

    fun editDirty(): Boolean {
        return selectedLibraryMap != editBaselineMap ||
            regions.toList() != editBaselineRegions ||
            draftVertices.isNotEmpty() ||
            redrawRegionId != null ||
            pendingDraftName
    }

    fun beginEditSession() {
        editBaselineMap = selectedLibraryMap
        editBaselineRegions = regions.toList()
        selectedRegionId = null
        draftVertices = emptyList()
        redrawRegionId = null
        pendingDraftName = false
        workMode = K1MapWorkMode.EDIT
        if (endpointKind == K1EndpointKind.BRIDGE) {
            refreshLibraryMaps()
        }
    }

    fun cancelEditChanges() {
        selectedLibraryMap = editBaselineMap
        regions.clear()
        regions.addAll(editBaselineRegions)
        selectedRegionId = null
        draftVertices = emptyList()
        redrawRegionId = null
        pendingDraftName = false
        nameDialogVisible = false
        regionStatus = "Edit changes cancelled"
    }

    fun leaveEditDiscardingChanges() {
        cancelEditChanges()
        workMode = K1MapWorkMode.DRIVE
        leaveEditConfirmVisible = false
    }

    fun leaveEditAfterSave() {
        if (trySaveRegionsToBoard()) {
            editBaselineMap = selectedLibraryMap
            editBaselineRegions = regions.toList()
            selectedRegionId = null
            draftVertices = emptyList()
            redrawRegionId = null
            pendingDraftName = false
            workMode = K1MapWorkMode.DRIVE
            leaveEditConfirmVisible = false
        }
    }

    fun leaveScreen() {
        cleanupAndDisconnect("Back: cleaning K1 processes", onBack)
    }

    val latestCleanup = rememberUpdatedState<(String) -> Unit> { reason ->
        cleanupAndDisconnect(reason)
    }

    DisposableEffect(lifecycleOwner) {
        val observer = LifecycleEventObserver { _, event ->
            if (event == Lifecycle.Event.ON_STOP && latestConnected) {
                latestCleanup.value("App background: cleaning K1 processes")
            }
        }
        lifecycleOwner.lifecycle.addObserver(observer)
        onDispose {
            lifecycleOwner.lifecycle.removeObserver(observer)
            if (latestConnected) {
                latestCleanup.value("Screen disposed: cleaning K1 processes")
            } else {
                bluetoothClient.disconnect()
            }
        }
    }

    LaunchedEffect(Unit) {
        while (true) {
            val manualTeleopActive = endpointKind == K1EndpointKind.BRIDGE &&
                manualEnabled &&
                !autoMappingActive
            if (connected && !connecting && !cleanupStarted && !manualTeleopActive) {
                bluetoothClient.sendAsync(
                    K1MobileProtocol.bridgeControl(nextSeq(), K1BridgeCommand.QUERY_BRIDGE)
                ) { message ->
                    addLog("Bridge monitor failed: $message")
                }
                if (endpointKind == K1EndpointKind.BRIDGE) {
                    val pending = pendingMapCommand ?: activeMapCommand
                    val mapTransitioning = mapState == K1MapState.STARTING ||
                        mapState == K1MapState.SAVING ||
                        mapState == K1MapState.STOPPING
                    if (pending != null && mapTransitioning) {
                        val now = SystemClock.elapsedRealtime()
                        if (now - pendingMapSentAt > 2500L) {
                            pendingMapSentAt = now
                            bluetoothClient.sendAsync(
                                K1MobileProtocol.mapControl(nextSeq(), K1MapCommand.QUERY_MAPPING)
                            ) { message ->
                                addLog("Map monitor failed: $message")
                            }
                        }
                    } else {
                        bluetoothClient.sendAsync(
                            K1MobileProtocol.mapControl(nextSeq(), K1MapCommand.QUERY_MAPPING)
                        ) { message ->
                            addLog("Map monitor failed: $message")
                        }
                    }
                }
            }
            delay(1500L)
        }
    }

    LaunchedEffect(connectAttemptToken) {
        if (connectAttemptToken > 0) {
            val token = connectAttemptToken
            delay(3500L)
            if (token == connectAttemptToken &&
                connected &&
                connectionPhase == K1ConnectionPhase.WAIT_HELLO &&
                endpointKind == K1EndpointKind.UNKNOWN &&
                !cleanupStarted
            ) {
                addLog("K1 handshake timeout; reconnecting")
                bluetoothClient.disconnect {
                    connected = false
                    connecting = false
                    connectionPhase = K1ConnectionPhase.RECOVERING
                    endpointKind = K1EndpointKind.UNKNOWN
                    bridgeBusy = false
                    mapBusy = false
                    resetTeleopInputs()
                    autoReconnectToken += 1
                }
            }
        }
    }

    LaunchedEffect(Unit) {
        while (true) {
            if (latestConnected &&
                latestEndpointKind == K1EndpointKind.BRIDGE &&
                latestManualEnabled &&
                !latestAutoMappingActive
            ) {
                // Dedicated teleop executor — never queues behind map/bridge writes
                bluetoothClient.sendTeleopAsync(
                    K1MobileProtocol.teleop(nextSeq(), latestVx, latestVy, latestOmega)
                )
                delay(50L)
            } else {
                delay(100L)
            }
        }
    }

    LaunchedEffect(autoReconnectToken) {
        if (autoReconnectToken > 0) {
            var attempt = 0
            val maxAttempts = 5
            while (attempt < maxAttempts &&
                endpointKind == K1EndpointKind.UNKNOWN &&
                connectionPhase != K1ConnectionPhase.DISCONNECTING &&
                !cleanupStarted
            ) {
                attempt++
                val delayMs = 2000L + (attempt - 1) * 1500L
                if (attempt > 1) {
                    addLog("Reconnection attempt $attempt/$maxAttempts")
                }
                delay(delayMs)
                if (!connected && !connecting) {
                    connectSelected()
                }
                val pollStart = SystemClock.uptimeMillis()
                while ((connecting || connectionPhase == K1ConnectionPhase.WAIT_HELLO) &&
                    endpointKind == K1EndpointKind.UNKNOWN &&
                    SystemClock.uptimeMillis() - pollStart < 6000L
                ) {
                    delay(200L)
                }
            }
            if (endpointKind == K1EndpointKind.UNKNOWN && !cleanupStarted) {
                addLog("K1 bridge reconnection failed after $maxAttempts attempts")
                connected = false
                connecting = false
                connectionPhase = K1ConnectionPhase.DISCONNECTED
                endpointKind = K1EndpointKind.UNKNOWN
                bridgeBusy = false
                reconnectAfterBridgeStart = false
            }
        }
    }

    LaunchedEffect(devices.size) {
        if (selectedAddress == null && devices.isNotEmpty()) {
            preferredK1MapDevice(devices)?.let { device ->
                selectedAddress = device.address
                selectedName = device.name
            }
        }
    }

    BoxWithConstraints(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFF0F1217))
            .padding(8.dp)
    ) {
        val compactLayout = maxWidth < 820.dp
        val topSpacing = if (compactLayout) 5.dp else 8.dp
        val backWidth = if (compactLayout) 56.dp else 64.dp
        val selectorWidth = if (compactLayout) 176.dp else 230.dp
        val resetWidth = if (compactLayout) 58.dp else 66.dp
        val mapSummary = mapBuffer.info?.let { "${it.width}x${it.height}" } ?: "none"
        val poseSummary = mapBuffer.pose?.let { "x ${it.x.format2()} y ${it.y.format2()}\nyaw ${it.yaw.format2()}" } ?: "none"
        val onlineBridge = connected && endpointKind == K1EndpointKind.BRIDGE
        val supervisor = connected && endpointKind == K1EndpointKind.SUPERVISOR
        val active = bridgeBusy || mapBusy || connecting || cleanupStarted

        K1LandscapeDriveLayout(
            devices = devices,
            selectedAddress = selectedAddress,
            selectedName = selectedName,
            connected = connected,
            connecting = connecting,
            hasBluetoothPermission = hasBluetoothPermission,
            endpointKind = endpointKind,
            bridgeState = bridgeState,
            mapState = mapState,
            bridgeBusy = bridgeBusy,
            mapBusy = mapBusy,
            cleanupStarted = cleanupStarted,
            reconnectAfterBridgeStart = reconnectAfterBridgeStart,
            pendingMapCommand = pendingMapCommand,
            activeMapCommand = activeMapCommand,
            mapSummary = mapSummary,
            poseSummary = poseSummary,
            manualEnabled = manualEnabled,
            manualLocked = autoMappingActive,
            mapStatus = mapBuffer.status,
            logs = logs,
            speedScale = speedScale,
            omegaScale = omegaScale,
            vx = vx,
            vy = vy,
            omega = omega,
            workMode = workMode,
            libraryMaps = libraryMaps,
            selectedLibraryMap = selectedLibraryMap,
            regions = regions,
            selectedRegionId = selectedRegionId,
            draftPointCount = draftVertices.size,
            libraryBusy = libraryBusy,
            regionStatus = regionStatus,
            compactLayout = compactLayout,
            onBack = { leaveScreen() },
            onRequestPermission = onRequestPermission,
            onRefreshDevices = onRefreshDevices,
            onSelectDevice = { device ->
                selectedAddress = device.address
                selectedName = device.name
            },
            onConnectToggle = {
                if (connected || connecting) {
                    if (connecting && !connected) {
                        bluetoothClient.disconnect {
                            resetDisconnectedState()
                            addLog("Connection cancelled")
                        }
                    } else {
                        cleanupAndDisconnect("Off: cleaning K1 processes")
                    }
                } else {
                    connectSelected()
                }
            },
            onStartBridge = { sendBridgeCommand(K1BridgeCommand.START_BRIDGE) },
            onStopBridge = { cleanupAndDisconnect("Stop Bridge: cleaning K1 processes") },
            onStartMapping = { mapStartModeDialogVisible = true },
            onSaveMapping = { sendMapCommand(K1MapCommand.SAVE_MAP_MANUAL) },
            onStopMapping = { sendMapCommand(K1MapCommand.STOP_MAPPING) },
            onResetView = { resetViewToken += 1 },
            onRotateMap = { mapRotationTurns = (mapRotationTurns + 1) and 3 },
            onToggleWorkMode = {
                if (workMode == K1MapWorkMode.DRIVE) {
                    if (manualEnabled) {
                        sendK1Stop("Edit mode")
                    }
                    manualEnabled = false
                    beginEditSession()
                } else {
                    if (editDirty()) {
                        leaveEditConfirmVisible = true
                    } else {
                        workMode = K1MapWorkMode.DRIVE
                    }
                }
            },
            onRefreshLibraryMaps = { refreshLibraryMaps() },
            onSelectLibraryMap = { selectedLibraryMap = it },
            onLoadLibraryMap = { selectedLibraryMap?.let(::loadLibraryMap) },
            onAddRegion = { startAddRegion() },
            onUndoRegionPoint = {
                if (draftVertices.isNotEmpty()) {
                    draftVertices = draftVertices.dropLast(1)
                }
            },
            onCloseRegion = { completeDraftRegion() },
            onRenameRegion = { renameSelectedRegion() },
            onDeleteRegion = { if (selectedRegionId != null) deleteConfirmVisible = true },
            onRedrawRegion = { redrawSelectedRegion() },
            onSaveRegions = {
                if (trySaveRegionsToBoard()) {
                    editBaselineMap = selectedLibraryMap
                    editBaselineRegions = regions.toList()
                    draftVertices = emptyList()
                    redrawRegionId = null
                    pendingDraftName = false
                }
            },
            onCancelEditChanges = { cancelEditChanges() },
            onSelectRegion = { selectedRegionId = it },
            onOpenLog = { logExpanded = true },
            onManualToggle = {
                if (autoMappingActive) {
                    manualEnabled = false
                    resetTeleopInputs()
                    addLog("Manual disabled during auto mapping")
                } else if (manualEnabled) {
                    manualEnabled = false
                    sendK1Stop("Manual off")
                } else {
                    sendK1Stop("Manual on")
                    manualEnabled = true
                }
            },
            onSpeedScaleChange = {
                speedScale = it
                addLog("Speed scale ${(it * 100).toInt()}%")
            },
            onOmegaScaleChange = {
                omegaScale = it
                addLog("Angular scale ${(it * 100).toInt()}%")
            },
            onJoystickChange = { forward, strafe ->
                vx = forward
                vy = strafe
            },
            onOmegaChange = { omega = it },
            mapContent = {
                Box(Modifier.fillMaxSize()) {
                    K1MapCanvas(
                        mapBuffer = mapBuffer,
                        resetToken = resetViewToken,
                        isLastMap = mapBuffer.isLastMap,
                        rotationTurns = mapRotationTurns,
                        regions = regions,
                        selectedRegionId = selectedRegionId,
                        draftVertices = draftVertices,
                        editingEnabled = workMode == K1MapWorkMode.EDIT && selectedLibraryMap != null,
                        showNavPath = autoMappingActive,
                        onMapTap = { point ->
                            if (pendingDraftName || redrawRegionId != null) {
                                draftVertices = draftVertices + point
                                regionStatus = "${draftVertices.size} points"
                            } else {
                                val hit = regions.asReversed().firstOrNull { region ->
                                    pointInPolygon(point, region.vertices)
                                }
                                selectedRegionId = hit?.id
                                regionStatus = hit?.let { "Selected ${it.name}" } ?: "No region selected"
                            }
                        },
                        modifier = Modifier.fillMaxSize()
                    )
                }
            },
            modifier = Modifier.fillMaxSize()
        )

        if (nameDialogVisible) {
            K1RegionNameDialog(
                value = nameDialogText,
                onValueChange = { nameDialogText = it },
                onDismiss = { nameDialogVisible = false },
                onConfirm = {
                    val trimmed = nameDialogText.trim()
                    if (trimmed.isNotEmpty()) {
                        if (closeable(draftVertices)) {
                            commitDraftRegion(trimmed)
                        } else {
                            val target = selectedRegionId
                            val index = regions.indexOfFirst { it.id == target }
                            if (index >= 0) {
                                regions[index] = regions[index].copy(name = trimmed)
                                regionStatus = "Region renamed locally"
                            }
                            nameDialogVisible = false
                        }
                    }
                }
            )
        }

        if (leaveEditConfirmVisible) {
            K1LeaveEditDialog(
                onSave = { leaveEditAfterSave() },
                onDiscard = { leaveEditDiscardingChanges() },
                onStay = { leaveEditConfirmVisible = false }
            )
        }

        if (deleteConfirmVisible) {
            val targetName = regions.firstOrNull { it.id == selectedRegionId }?.name.orEmpty()
            K1DeleteRegionDialog(
                regionName = targetName,
                onCancel = { deleteConfirmVisible = false },
                onConfirm = {
                    deleteConfirmVisible = false
                    deleteSelectedRegion()
                }
            )
        }

        if (logExpanded) {
            K1LogPopup(
                logs = logs,
                onDismiss = { logExpanded = false },
                modifier = Modifier
                    .align(Alignment.TopEnd)
                    .zIndex(8f)
            )
        }

        if (mapStartModeDialogVisible) {
            K1StartMapModeDialog(
                onDismiss = { mapStartModeDialogVisible = false },
                onManual = {
                    mapStartModeDialogVisible = false
                    sendMapCommand(K1MapCommand.START_MAPPING, K1MappingMode.MANUAL)
                },
                onAuto = {
                    mapStartModeDialogVisible = false
                    selectedAutoRoomSize = K1AutoMapRoomSize.MEDIUM
                    autoCustomDialogVisible = false
                    autoRoomDialogVisible = true
                }
            )
        }

        if (autoRoomDialogVisible) {
            val customX = parseCustomExploreSize(customAutoXText)
            val customY = parseCustomExploreSize(customAutoYText)
            K1AutoRoomDialog(
                selected = selectedAutoRoomSize,
                onSelect = { roomSize ->
                    if (roomSize == K1AutoMapRoomSize.CUSTOM) {
                        selectedAutoRoomSize = roomSize
                        autoRoomDialogVisible = false
                        autoCustomDialogVisible = true
                    } else {
                        selectedAutoRoomSize = roomSize
                    }
                },
                customXText = customAutoXText,
                customYText = customAutoYText,
                onCustomXChange = { customAutoXText = it },
                onCustomYChange = { customAutoYText = it },
                customInputValid = selectedAutoRoomSize != K1AutoMapRoomSize.CUSTOM ||
                    (customX != null && customY != null),
                onStart = {
                    if (selectedAutoRoomSize == K1AutoMapRoomSize.CUSTOM &&
                        (customX == null || customY == null)
                    ) {
                        addLog("Custom area must be 1-30 m")
                        return@K1AutoRoomDialog
                    }
                    autoRoomDialogVisible = false
                    sendMapCommand(
                        K1MapCommand.START_MAPPING,
                        K1MappingMode.AUTO,
                        selectedAutoRoomSize,
                        customX,
                        customY
                    )
                },
                onDismiss = { autoRoomDialogVisible = false }
            )
        }

        if (autoCustomDialogVisible) {
            val customX = parseCustomExploreSize(customAutoXText)
            val customY = parseCustomExploreSize(customAutoYText)
            K1CustomAreaDialog(
                customXText = customAutoXText,
                customYText = customAutoYText,
                onCustomXChange = { customAutoXText = it },
                onCustomYChange = { customAutoYText = it },
                customInputValid = customX != null && customY != null,
                onStart = {
                    if (customX == null || customY == null) {
                        addLog("Custom area must be 1-30 m")
                        return@K1CustomAreaDialog
                    }
                    autoCustomDialogVisible = false
                    sendMapCommand(
                        K1MapCommand.START_MAPPING,
                        K1MappingMode.AUTO,
                        K1AutoMapRoomSize.CUSTOM,
                        customX,
                        customY
                    )
                },
                onBack = {
                    autoCustomDialogVisible = false
                    autoRoomDialogVisible = true
                },
                onDismiss = { autoCustomDialogVisible = false }
            )
        }
    }
}

@Composable
private fun K1StartMapModeDialog(
    onDismiss: () -> Unit,
    onManual: () -> Unit,
    onAuto: () -> Unit
) {
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Start Map") },
        text = { Text("Choose mapping mode.") },
        confirmButton = {
            TextButton(
                onClick = onAuto,
                modifier = Modifier.testTag("k1-start-auto-map")
            ) {
                Text("Auto Mapping")
            }
        },
        dismissButton = {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                TextButton(
                    onClick = onManual,
                    modifier = Modifier.testTag("k1-start-manual-map")
                ) {
                    Text("Manual Mapping")
                }
                TextButton(onClick = onDismiss) {
                    Text("Cancel")
                }
            }
        },
        modifier = Modifier.testTag("k1-map-start-mode-dialog")
    )
}

@Composable
private fun K1AutoRoomDialog(
    selected: K1AutoMapRoomSize,
    onSelect: (K1AutoMapRoomSize) -> Unit,
    customXText: String,
    customYText: String,
    onCustomXChange: (String) -> Unit,
    onCustomYChange: (String) -> Unit,
    customInputValid: Boolean,
    onStart: () -> Unit,
    onDismiss: () -> Unit
) {
    AlertDialog(
        onDismissRequest = onDismiss,
        title = {
            Text(
                "Auto Area",
                fontSize = 24.sp,
                lineHeight = 26.sp
            )
        },
        text = {
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text(
                    "Select a preset or enter X/Y meters.",
                    fontSize = 13.sp,
                    lineHeight = 15.sp
                )
                listOf(
                    listOf(K1AutoMapRoomSize.SMALL, K1AutoMapRoomSize.MEDIUM),
                    listOf(K1AutoMapRoomSize.LARGE, K1AutoMapRoomSize.CUSTOM)
                ).forEach { rowItems ->
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        rowItems.forEach { roomSize ->
                            K1AutoRoomButton(
                                roomSize = roomSize,
                                selected = selected,
                                onSelect = onSelect,
                                modifier = Modifier.weight(1f)
                            )
                        }
                    }
                }
                if (selected == K1AutoMapRoomSize.CUSTOM) {
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        OutlinedTextField(
                            value = customXText,
                            onValueChange = onCustomXChange,
                            label = { Text("X meters") },
                            singleLine = true,
                            isError = parseCustomExploreSize(customXText) == null,
                            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
                            modifier = Modifier
                                .weight(1f)
                                .testTag("k1-auto-custom-x")
                        )
                        OutlinedTextField(
                            value = customYText,
                            onValueChange = onCustomYChange,
                            label = { Text("Y meters") },
                            singleLine = true,
                            isError = parseCustomExploreSize(customYText) == null,
                            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
                            modifier = Modifier
                                .weight(1f)
                                .testTag("k1-auto-custom-y")
                        )
                    }
                    if (!customInputValid) {
                        Text(
                            text = "Use 1-30 m for both X and Y.",
                            color = Color(0xFFFFD166),
                            fontSize = 11.sp,
                            lineHeight = 12.sp
                        )
                    }
                }
            }
        },
        confirmButton = {
            TextButton(
                onClick = onStart,
                enabled = customInputValid,
                modifier = Modifier.testTag("k1-auto-room-confirm")
            ) {
                Text("Start")
            }
        },
        dismissButton = {
            TextButton(onClick = onDismiss) {
                Text("Cancel")
            }
        },
        modifier = Modifier.testTag("k1-auto-room-dialog")
    )
}

private fun parseCustomExploreSize(value: String): Float? {
    val parsed = value.trim().toFloatOrNull() ?: return null
    return if (parsed in 1f..30f) parsed else null
}

@Composable
private fun K1CustomAreaDialog(
    customXText: String,
    customYText: String,
    onCustomXChange: (String) -> Unit,
    onCustomYChange: (String) -> Unit,
    customInputValid: Boolean,
    onStart: () -> Unit,
    onBack: () -> Unit,
    onDismiss: () -> Unit
) {
    AlertDialog(
        onDismissRequest = onDismiss,
        title = {
            Text(
                "Custom Area",
                fontSize = 24.sp,
                lineHeight = 26.sp
            )
        },
        text = {
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text(
                    "Input total X/Y size in meters. The board uses -X/2..X/2 and -Y/2..Y/2.",
                    fontSize = 13.sp,
                    lineHeight = 15.sp
                )
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    OutlinedTextField(
                        value = customXText,
                        onValueChange = onCustomXChange,
                        label = { Text("X meters") },
                        singleLine = true,
                        isError = parseCustomExploreSize(customXText) == null,
                        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
                        modifier = Modifier
                            .weight(1f)
                            .testTag("k1-auto-custom-x")
                    )
                    OutlinedTextField(
                        value = customYText,
                        onValueChange = onCustomYChange,
                        label = { Text("Y meters") },
                        singleLine = true,
                        isError = parseCustomExploreSize(customYText) == null,
                        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
                        modifier = Modifier
                            .weight(1f)
                            .testTag("k1-auto-custom-y")
                    )
                }
                if (!customInputValid) {
                    Text(
                        text = "Use 1-30 m for both X and Y.",
                        color = Color(0xFFFFD166),
                        fontSize = 11.sp,
                        lineHeight = 12.sp
                    )
                }
            }
        },
        confirmButton = {
            TextButton(
                onClick = onStart,
                enabled = customInputValid,
                modifier = Modifier.testTag("k1-auto-custom-start")
            ) {
                Text("Start")
            }
        },
        dismissButton = {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                TextButton(onClick = onBack) {
                    Text("Back")
                }
                TextButton(onClick = onDismiss) {
                    Text("Cancel")
                }
            }
        },
        modifier = Modifier.testTag("k1-auto-custom-dialog")
    )
}

@Composable
private fun K1AutoRoomButton(
    roomSize: K1AutoMapRoomSize,
    selected: K1AutoMapRoomSize,
    onSelect: (K1AutoMapRoomSize) -> Unit,
    modifier: Modifier = Modifier
) {
    Button(
        onClick = { onSelect(roomSize) },
        modifier = modifier
            .height(48.dp)
            .testTag("k1-auto-room-${roomSize.name.lowercase()}"),
        contentPadding = PaddingValues(horizontal = 6.dp),
        colors = ButtonDefaults.buttonColors(
            containerColor = if (roomSize == selected) Color(0xFF4F8CFF) else Color(0xFF232A34),
            contentColor = Color.White
        ),
        shape = RoundedCornerShape(6.dp)
    ) {
        Text(
            text = "${roomSize.label}\n${roomSize.detail}",
            fontSize = 9.sp,
            lineHeight = 10.sp,
            fontWeight = FontWeight.Bold,
            textAlign = TextAlign.Center
        )
    }
}

@Composable
private fun K1LandscapeDriveLayout(
    devices: List<BondedDeviceInfo>,
    selectedAddress: String?,
    selectedName: String,
    connected: Boolean,
    connecting: Boolean,
    hasBluetoothPermission: Boolean,
    endpointKind: K1EndpointKind,
    bridgeState: K1BridgeState,
    mapState: K1MapState,
    bridgeBusy: Boolean,
    mapBusy: Boolean,
    cleanupStarted: Boolean,
    reconnectAfterBridgeStart: Boolean,
    pendingMapCommand: K1MapCommand?,
    activeMapCommand: K1MapCommand?,
    mapSummary: String,
    poseSummary: String,
    manualEnabled: Boolean,
    manualLocked: Boolean,
    mapStatus: String,
    logs: List<String>,
    speedScale: Float,
    omegaScale: Float,
    vx: Float,
    vy: Float,
    omega: Float,
    workMode: K1MapWorkMode,
    libraryMaps: List<K1MapLibraryEntry>,
    selectedLibraryMap: K1MapLibraryEntry?,
    regions: List<MapRegion>,
    selectedRegionId: String?,
    draftPointCount: Int,
    libraryBusy: Boolean,
    regionStatus: String,
    compactLayout: Boolean,
    onBack: () -> Unit,
    onRequestPermission: () -> Unit,
    onRefreshDevices: () -> Unit,
    onSelectDevice: (BondedDeviceInfo) -> Unit,
    onConnectToggle: () -> Unit,
    onStartBridge: () -> Unit,
    onStopBridge: () -> Unit,
    onStartMapping: () -> Unit,
    onSaveMapping: () -> Unit,
    onStopMapping: () -> Unit,
    onResetView: () -> Unit,
    onRotateMap: () -> Unit,
    onToggleWorkMode: () -> Unit,
    onRefreshLibraryMaps: () -> Unit,
    onSelectLibraryMap: (K1MapLibraryEntry) -> Unit,
    onLoadLibraryMap: () -> Unit,
    onAddRegion: () -> Unit,
    onUndoRegionPoint: () -> Unit,
    onCloseRegion: () -> Unit,
    onRenameRegion: () -> Unit,
    onDeleteRegion: () -> Unit,
    onRedrawRegion: () -> Unit,
    onSaveRegions: () -> Unit,
    onCancelEditChanges: () -> Unit,
    onSelectRegion: (String) -> Unit,
    onOpenLog: () -> Unit,
    onManualToggle: () -> Unit,
    onSpeedScaleChange: (Float) -> Unit,
    onOmegaScaleChange: (Float) -> Unit,
    onJoystickChange: (Float, Float) -> Unit,
    onOmegaChange: (Float) -> Unit,
    mapContent: @Composable () -> Unit,
    modifier: Modifier = Modifier
) {
    val topSpacing = if (compactLayout) 5.dp else 8.dp
    val onlineBridge = connected && endpointKind == K1EndpointKind.BRIDGE
    val active = bridgeBusy || mapBusy || connecting || cleanupStarted
    val bridgeButtonMode = bridgeActionMode(
        connected = connected,
        endpointKind = endpointKind,
        bridgeState = bridgeState,
        mapState = mapState,
        bridgeBusy = bridgeBusy,
        mapBusy = mapBusy,
        connecting = connecting,
        cleanupStarted = cleanupStarted,
        reconnectAfterBridgeStart = reconnectAfterBridgeStart
    )
    val mapButtonMode = mapActionMode(
        onlineBridge = onlineBridge,
        mapState = mapState,
        bridgeBusy = bridgeBusy,
        mapBusy = mapBusy,
        connecting = connecting,
        cleanupStarted = cleanupStarted,
        pendingMapCommand = pendingMapCommand,
        activeMapCommand = activeMapCommand
    )
    val bridgeLabel = when (endpointKind) {
        K1EndpointKind.SUPERVISOR -> if (bridgeState == K1BridgeState.STARTING) "starting" else "supervisor"
        K1EndpointKind.BRIDGE -> bridgeState.name.lowercase()
        K1EndpointKind.UNKNOWN -> if (connected) "detecting" else "offline"
    }

        Column(
            modifier = modifier,
            verticalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(72.dp),
                verticalArrangement = Arrangement.spacedBy(4.dp)
            ) {
                Row(
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(36.dp),
                    horizontalArrangement = Arrangement.spacedBy(topSpacing),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                Button(
                    onClick = onBack,
                    modifier = Modifier
                        .width(if (compactLayout) 54.dp else 62.dp)
                        .height(34.dp),
                    contentPadding = PaddingValues(0.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFF232A34),
                        contentColor = Color.White
                    )
                ) {
                    Text("Back", fontSize = 10.sp, fontWeight = FontWeight.Bold)
                }

                K1DeviceSelector(
                    devices = devices,
                    selectedAddress = selectedAddress,
                    selectedName = selectedName,
                    connected = connected,
                    connecting = connecting,
                    hasBluetoothPermission = hasBluetoothPermission,
                    onRequestPermission = onRequestPermission,
                    onRefreshDevices = onRefreshDevices,
                    onSelectDevice = onSelectDevice,
                    onConnectToggle = onConnectToggle,
                    modifier = Modifier.width(if (compactLayout) 174.dp else 220.dp)
                )

                StatusChip("K1", bridgeLabel, Modifier.width(78.dp))
                StatusChip("MAP", mapSummary, Modifier.width(68.dp))
                StatusChip("POSE", poseSummary, Modifier.width(if (compactLayout) 106.dp else 118.dp))
                StatusChip("MANUAL", if (manualLocked) "AUTO" else if (manualEnabled) "ON" else "OFF", Modifier.width(66.dp))
                K1LogPreview(
                    logs = logs,
                    status = mapStatus,
                    onClick = onOpenLog,
                    modifier = Modifier.weight(1f)
                )
            }

                Row(
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(30.dp),
                    horizontalArrangement = Arrangement.spacedBy(topSpacing),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                K1TopActionButton(
                    bridgeButtonMode.text,
                    bridgeButtonMode.clickEnabled,
                    bridgeButtonMode.color,
                    {
                        if (endpointKind == K1EndpointKind.SUPERVISOR) {
                            onStartBridge()
                        } else if (onlineBridge) {
                            onStopBridge()
                        }
                    },
                    Modifier.width(104.dp),
                    holdColorWhenDisabled = true
                )
                K1TopActionButton(
                    mapButtonMode.text,
                    mapButtonMode.clickEnabled,
                    mapButtonMode.color,
                    {
                        if (mapState == K1MapState.MAPPING) {
                            onStopMapping()
                        } else {
                            onStartMapping()
                        }
                    },
                    Modifier.width(96.dp),
                    holdColorWhenDisabled = true
                )
                K1TopActionButton(
                    "Save Map",
                    onlineBridge && !active && mapState == K1MapState.MAPPING,
                    Color(0xFFE6A23A),
                    onSaveMapping,
                    Modifier.width(76.dp),
                    holdColorWhenDisabled = onlineBridge && mapState == K1MapState.MAPPING
                )
                Button(
                    onClick = onResetView,
                    modifier = Modifier
                        .width(if (compactLayout) 62.dp else 70.dp)
                        .height(28.dp),
                    contentPadding = PaddingValues(0.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFF232A34),
                        contentColor = Color.White
                    )
                ) {
                    Text("Reset View", fontSize = 9.sp, lineHeight = 10.sp, fontWeight = FontWeight.Bold)
                }
                Button(
                    onClick = onRotateMap,
                    modifier = Modifier
                        .width(if (compactLayout) 54.dp else 62.dp)
                        .height(28.dp)
                        .testTag("k1-map-rotate"),
                    contentPadding = PaddingValues(0.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFF232A34),
                        contentColor = Color.White
                    )
                ) {
                    Text("Rotate", fontSize = 9.sp, lineHeight = 10.sp, fontWeight = FontWeight.Bold)
                }
                Button(
                    onClick = onToggleWorkMode,
                    enabled = onlineBridge && !active,
                    modifier = Modifier
                        .width(if (compactLayout) 62.dp else 70.dp)
                        .height(28.dp)
                        .testTag("k1-work-mode-toggle"),
                    contentPadding = PaddingValues(0.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (workMode == K1MapWorkMode.EDIT) Color(0xFF20A66B) else Color(0xFF232A34),
                        contentColor = Color.White,
                        disabledContainerColor = Color(0xFF3A414D),
                        disabledContentColor = Color(0xFFB9C6D6)
                    )
                ) {
                    Text(
                        if (workMode == K1MapWorkMode.EDIT) "Edit" else "Drive",
                        fontSize = 9.sp,
                        lineHeight = 10.sp,
                        fontWeight = FontWeight.Bold
                    )
                }
                Spacer(Modifier.weight(1f))
            }
        }

        Row(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
                if (workMode == K1MapWorkMode.EDIT) {
                    K1RegionEditOverlay(
                        libraryMaps = libraryMaps,
                        selectedLibraryMap = selectedLibraryMap,
                        regions = regions,
                        selectedRegionId = selectedRegionId,
                        draftPointCount = draftPointCount,
                        onlineBridge = onlineBridge,
                        libraryBusy = libraryBusy,
                        regionStatus = regionStatus,
                        onRefreshLibraryMaps = onRefreshLibraryMaps,
                        onSelectLibraryMap = onSelectLibraryMap,
                        onLoadLibraryMap = onLoadLibraryMap,
                        onAddRegion = onAddRegion,
                        onUndoRegionPoint = onUndoRegionPoint,
                        onCloseRegion = onCloseRegion,
                        onRenameRegion = onRenameRegion,
                        onDeleteRegion = onDeleteRegion,
                        onRedrawRegion = onRedrawRegion,
                        onSaveRegions = onSaveRegions,
                        onCancelEditChanges = onCancelEditChanges,
                        onSelectRegion = onSelectRegion,
                        modifier = Modifier
                            .width(if (compactLayout) 156.dp else 170.dp)
                            .fillMaxHeight()
                    )
                } else {
                    K1LeftDriveRail(
                    connected = onlineBridge,
                    manualEnabled = manualEnabled,
                    speedScale = speedScale,
                    vx = vx,
                    vy = vy,
                    onJoystickChange = onJoystickChange,
                        modifier = Modifier
                            .width(if (compactLayout) 156.dp else 170.dp)
                            .fillMaxHeight()
                    )
                }

            Box(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxHeight()
            ) {
                mapContent()
            }

                K1RightDriveRail(
                connected = onlineBridge,
                manualEnabled = manualEnabled,
                manualLocked = manualLocked,
                speedScale = speedScale,
                omegaScale = omegaScale,
                omega = omega,
                onManualToggle = onManualToggle,
                onSpeedScaleChange = onSpeedScaleChange,
                onOmegaScaleChange = onOmegaScaleChange,
                onOmegaChange = onOmegaChange,
                    modifier = Modifier
                        .width(if (compactLayout) 156.dp else 170.dp)
                        .fillMaxHeight()
                )
        }
    }
}

@Composable
private fun K1TopActionButton(
    text: String,
    enabled: Boolean,
    color: Color,
    onClick: () -> Unit,
    modifier: Modifier = Modifier,
    holdColorWhenDisabled: Boolean = false
) {
    Button(
        onClick = onClick,
        enabled = enabled,
        modifier = modifier.height(28.dp),
        contentPadding = PaddingValues(0.dp),
        colors = ButtonDefaults.buttonColors(
            containerColor = color,
            contentColor = Color.White,
            disabledContainerColor = if (holdColorWhenDisabled) color else Color(0xFF3A414D),
            disabledContentColor = if (holdColorWhenDisabled) Color.White else Color(0xFFB9C6D6)
        ),
        shape = RoundedCornerShape(50)
    ) {
        Text(
            text = text,
            fontSize = 6.sp,
            lineHeight = 7.sp,
            fontWeight = FontWeight.Black,
            textAlign = TextAlign.Center
        )
    }
}

@Composable
private fun K1LeftDriveRail(
    connected: Boolean,
    manualEnabled: Boolean,
    speedScale: Float,
    vx: Float,
    vy: Float,
    onJoystickChange: (Float, Float) -> Unit,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .clip(RoundedCornerShape(8.dp))
            .background(Color(0xE6111820))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(8.dp))
            .padding(8.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
        horizontalAlignment = Alignment.Start
    ) {
        Text(
            "DRIVE",
            color = Color(0xFFB3C6DE),
            fontSize = 8.sp,
            fontWeight = FontWeight.ExtraBold
        )
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            K1Metric("VX", vx.format2(), Modifier.weight(1f))
            K1Metric("VY", vy.format2(), Modifier.weight(1f))
        }
        Spacer(Modifier.weight(1f))
        K1JoystickControl(
            speedScale = speedScale,
            enabled = connected && manualEnabled,
            onChange = onJoystickChange,
            modifier = Modifier.size(136.dp)
        )
    }
}

@Composable
private fun K1RightDriveRail(
    connected: Boolean,
    manualEnabled: Boolean,
    manualLocked: Boolean,
    speedScale: Float,
    omegaScale: Float,
    omega: Float,
    onManualToggle: () -> Unit,
    onSpeedScaleChange: (Float) -> Unit,
    onOmegaScaleChange: (Float) -> Unit,
    onOmegaChange: (Float) -> Unit,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .clip(RoundedCornerShape(8.dp))
            .background(Color(0xE6111820))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(8.dp))
            .padding(8.dp),
        verticalArrangement = Arrangement.spacedBy(6.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Button(
            onClick = onManualToggle,
            enabled = connected && !manualLocked,
            modifier = Modifier
                .fillMaxWidth()
                .height(40.dp),
            contentPadding = PaddingValues(0.dp),
            colors = ButtonDefaults.buttonColors(
                containerColor = if (manualLocked) Color(0xFF3A414D) else if (manualEnabled) Color(0xFFE53935) else Color(0xFF20A66B),
                contentColor = Color.White,
                disabledContainerColor = Color(0xFF3A414D),
                disabledContentColor = Color(0xFFB9C6D6)
            ),
            shape = RoundedCornerShape(50)
        ) {
            Text(
                if (manualLocked) "AUTO MAP" else if (manualEnabled) "STOP MANUAL" else "ENABLE MANUAL",
                fontSize = 9.sp,
                fontWeight = FontWeight.Black
            )
        }
        K1ScalePanel("SPEED", speedScale, Color(0xFF4F8CFF), Modifier.fillMaxWidth(), onSpeedScaleChange)
        K1ScalePanel("ANGULAR", omegaScale, Color(0xFF4F8CFF), Modifier.fillMaxWidth(), onOmegaScaleChange)
        K1Metric("W", omega.format2(), Modifier.fillMaxWidth())
        Spacer(Modifier.weight(1f))
        K1OmegaSlider(
            omegaScale = omegaScale,
            enabled = connected && manualEnabled,
            onChange = onOmegaChange,
            modifier = Modifier
                .fillMaxWidth()
                .height(48.dp)
        )
    }
}

@Composable
internal fun K1RegionEditOverlay(
    libraryMaps: List<K1MapLibraryEntry>,
    selectedLibraryMap: K1MapLibraryEntry?,
    regions: List<MapRegion>,
    selectedRegionId: String?,
    draftPointCount: Int,
    onlineBridge: Boolean,
    libraryBusy: Boolean,
    regionStatus: String,
    onRefreshLibraryMaps: () -> Unit,
    onSelectLibraryMap: (K1MapLibraryEntry) -> Unit,
    onLoadLibraryMap: () -> Unit,
    onAddRegion: () -> Unit,
    onUndoRegionPoint: () -> Unit,
    onCloseRegion: () -> Unit,
    onRenameRegion: () -> Unit,
    onDeleteRegion: () -> Unit,
    onRedrawRegion: () -> Unit,
    onSaveRegions: () -> Unit,
    onCancelEditChanges: () -> Unit,
    onSelectRegion: (String) -> Unit,
    modifier: Modifier = Modifier
) {
    var mapMenuExpanded by remember { mutableStateOf(false) }
    val hasMap = selectedLibraryMap != null
    val hasSelection = selectedRegionId != null
    Column(
        modifier = modifier
            .clip(RoundedCornerShape(7.dp))
            .background(Color(0xE6111820))
            .border(1.dp, Color(0xFF4F8CFF), RoundedCornerShape(7.dp))
            .padding(8.dp)
            .testTag("k1-region-editor"),
        verticalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        Text("REGIONS", color = Color(0xFFB3C6DE), fontSize = 8.sp, fontWeight = FontWeight.ExtraBold)
        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            K1RegionButton("Maps", onlineBridge && !libraryBusy, onRefreshLibraryMaps, Modifier.weight(1f).testTag("regions-refresh-maps"))
            Box(Modifier.weight(1f)) {
                K1RegionButton(
                    selectedLibraryMap?.yamlName ?: "Select",
                    libraryMaps.isNotEmpty(),
                    { mapMenuExpanded = true },
                    Modifier.fillMaxWidth().testTag("regions-select-map")
                )
                DropdownMenu(expanded = mapMenuExpanded, onDismissRequest = { mapMenuExpanded = false }) {
                    libraryMaps.forEach { entry ->
                        DropdownMenuItem(
                            text = { Text(if (entry.hasRegions) "${entry.yamlName} *" else entry.yamlName) },
                            onClick = {
                                onSelectLibraryMap(entry)
                                mapMenuExpanded = false
                            }
                        )
                    }
                }
            }
        }
        K1RegionButton("Load Map", onlineBridge && hasMap && !libraryBusy, onLoadLibraryMap, Modifier.fillMaxWidth().testTag("regions-load-map"))
        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            K1RegionButton("Add", hasMap, onAddRegion, Modifier.weight(1f).testTag("regions-add"))
            K1RegionButton("Undo", draftPointCount > 0, onUndoRegionPoint, Modifier.weight(1f).testTag("regions-undo"))
            K1RegionButton("Close", closeable(List(draftPointCount) { RegionPoint(0f, 0f) }), onCloseRegion, Modifier.weight(1f).testTag("regions-close"))
        }
        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            K1RegionButton("Rename", hasSelection, onRenameRegion, Modifier.weight(1f).testTag("regions-rename"))
            K1RegionButton("Delete", hasSelection, onDeleteRegion, Modifier.weight(1f).testTag("regions-delete"))
            K1RegionButton("Redraw", hasSelection, onRedrawRegion, Modifier.weight(1f).testTag("regions-redraw"))
        }
        K1RegionButton("Save Regions", onlineBridge && hasMap && !libraryBusy, onSaveRegions, Modifier.fillMaxWidth().testTag("regions-save"))
        K1RegionButton("Cancel Edit", true, onCancelEditChanges, Modifier.fillMaxWidth().testTag("regions-cancel"))
        Text(
            text = "$regionStatus\npoints $draftPointCount",
            color = Color(0xFFD6E1EF),
            fontSize = 8.sp,
            lineHeight = 10.sp,
            maxLines = 3,
            overflow = TextOverflow.Clip
        )
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .height(92.dp)
                .verticalScroll(rememberScrollState())
                .testTag("regions-list"),
            verticalArrangement = Arrangement.spacedBy(4.dp)
        ) {
            if (regions.isEmpty()) {
                Text("No regions", color = Color(0xFFB9C6D6), fontSize = 8.sp)
            }
            regions.forEach { region ->
                Button(
                    onClick = { onSelectRegion(region.id) },
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(28.dp)
                        .testTag("region-row-${region.name}"),
                    contentPadding = PaddingValues(horizontal = 6.dp, vertical = 0.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (region.id == selectedRegionId) Color(0xFF4F8CFF) else Color(0xFF232A34),
                        contentColor = Color.White
                    ),
                    shape = RoundedCornerShape(6.dp)
                ) {
                    Text(region.name, fontSize = 8.sp, maxLines = 1, overflow = TextOverflow.Clip)
                }
            }
        }
    }
}

@Composable
private fun K1RegionButton(
    text: String,
    enabled: Boolean,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    Button(
        onClick = onClick,
        enabled = enabled,
        modifier = modifier.height(28.dp),
        contentPadding = PaddingValues(horizontal = 4.dp, vertical = 0.dp),
        colors = ButtonDefaults.buttonColors(
            containerColor = Color(0xFF232A34),
            contentColor = Color.White,
            disabledContainerColor = Color(0xFF3A414D),
            disabledContentColor = Color(0xFFB9C6D6)
        ),
        shape = RoundedCornerShape(6.dp)
    ) {
        Text(text, fontSize = 7.sp, lineHeight = 8.sp, maxLines = 1, overflow = TextOverflow.Clip)
    }
}

@Composable
private fun K1RegionNameDialog(
    value: String,
    onValueChange: (String) -> Unit,
    onDismiss: () -> Unit,
    onConfirm: () -> Unit
) {
    Box(
        modifier = Modifier
            .fillMaxSize()
            .zIndex(10f)
            .background(Color(0x99000000))
            .testTag("region-name-dialog"),
        contentAlignment = Alignment.Center
    ) {
        Column(
            modifier = Modifier
                .width(280.dp)
                .clip(RoundedCornerShape(8.dp))
                .background(Color(0xFF111820))
                .border(1.dp, Color(0xFF4F8CFF), RoundedCornerShape(8.dp))
                .padding(12.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Text("Region Name", color = Color.White, fontSize = 14.sp, fontWeight = FontWeight.Bold)
            OutlinedTextField(
                value = value,
                onValueChange = onValueChange,
                singleLine = true,
                modifier = Modifier.fillMaxWidth().testTag("region-name-input")
            )
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                TextButton(onClick = onDismiss, modifier = Modifier.weight(1f).testTag("region-name-cancel")) {
                    Text("Cancel")
                }
                Button(onClick = onConfirm, modifier = Modifier.weight(1f).testTag("region-name-ok")) {
                    Text("OK")
                }
            }
        }
    }
}

@Composable
private fun K1LeaveEditDialog(
    onSave: () -> Unit,
    onDiscard: () -> Unit,
    onStay: () -> Unit
) {
    Box(
        modifier = Modifier
            .fillMaxSize()
            .zIndex(10f)
            .background(Color(0x99000000))
            .testTag("leave-edit-dialog"),
        contentAlignment = Alignment.Center
    ) {
        Column(
            modifier = Modifier
                .width(310.dp)
                .clip(RoundedCornerShape(8.dp))
                .background(Color(0xFF111820))
                .border(1.dp, Color(0xFFE6A23A), RoundedCornerShape(8.dp))
                .padding(12.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Text("Save region changes?", color = Color.White, fontSize = 14.sp, fontWeight = FontWeight.Bold)
            Text(
                "Leaving Edit will discard unsaved region edits unless they are saved to K1.",
                color = Color(0xFFD6E1EF),
                fontSize = 10.sp,
                lineHeight = 12.sp
            )
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                TextButton(onClick = onStay, modifier = Modifier.weight(1f).testTag("leave-edit-stay")) {
                    Text("Stay")
                }
                TextButton(onClick = onDiscard, modifier = Modifier.weight(1f).testTag("leave-edit-discard")) {
                    Text("Discard")
                }
                Button(onClick = onSave, modifier = Modifier.weight(1f).testTag("leave-edit-save")) {
                    Text("Save")
                }
            }
        }
    }
}

@Composable
private fun K1DeleteRegionDialog(
    regionName: String,
    onCancel: () -> Unit,
    onConfirm: () -> Unit
) {
    Box(
        modifier = Modifier
            .fillMaxSize()
            .zIndex(10f)
            .background(Color(0x99000000))
            .testTag("delete-region-dialog"),
        contentAlignment = Alignment.Center
    ) {
        Column(
            modifier = Modifier
                .width(290.dp)
                .clip(RoundedCornerShape(8.dp))
                .background(Color(0xFF111820))
                .border(1.dp, Color(0xFFE53935), RoundedCornerShape(8.dp))
                .padding(12.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Text("Delete region?", color = Color.White, fontSize = 14.sp, fontWeight = FontWeight.Bold)
            Text(
                if (regionName.isBlank()) "This region will be removed locally." else "$regionName will be removed locally.",
                color = Color(0xFFD6E1EF),
                fontSize = 10.sp,
                lineHeight = 12.sp
            )
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                TextButton(onClick = onCancel, modifier = Modifier.weight(1f).testTag("delete-region-cancel")) {
                    Text("Cancel")
                }
                Button(onClick = onConfirm, modifier = Modifier.weight(1f).testTag("delete-region-ok")) {
                    Text("Delete")
                }
            }
        }
    }
}

@Composable
private fun K1LogPreview(
    logs: List<String>,
    status: String,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .height(34.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF111820))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(6.dp))
            .clickable(onClick = onClick)
            .padding(horizontal = 8.dp, vertical = 3.dp)
    ) {
        Text(
            text = "$status\n${logs.firstOrNull().orEmpty()}",
            color = Color(0xFFD6E1EF),
            fontSize = 6.sp,
            lineHeight = 7.sp,
            maxLines = 4,
            overflow = TextOverflow.Clip
        )
    }
}

@Composable
private fun K1LogPopup(
    logs: List<String>,
    onDismiss: () -> Unit,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = Modifier
            .fillMaxSize()
            .zIndex(7f)
            .pointerInput(Unit) {
                detectTapGestures { onDismiss() }
            }
    )
    Column(
        modifier = modifier
            .offset(x = (-18).dp, y = 56.dp)
            .width(430.dp)
            .height(238.dp)
            .clip(RoundedCornerShape(7.dp))
            .background(Color(0xFF111820))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(7.dp))
            .pointerInput(Unit) {
                detectTapGestures { }
            }
            .padding(horizontal = 10.dp, vertical = 8.dp)
    ) {
        Text(
            text = "K1 LOG",
            color = Color(0xFFB3C6DE),
            fontSize = 8.sp,
            lineHeight = 9.sp,
            fontWeight = FontWeight.ExtraBold,
            maxLines = 1
        )
        Spacer(Modifier.height(5.dp))
        Text(
            text = logs.joinToString("\n"),
            color = Color(0xFFD6E1EF),
            fontSize = 10.sp,
            lineHeight = 13.sp,
            overflow = TextOverflow.Clip,
            modifier = Modifier.verticalScroll(rememberScrollState())
        )
    }
}

@Composable
private fun K1MapCanvas(
    mapBuffer: MobileMapBuffer,
    resetToken: Int,
    isLastMap: Boolean,
    rotationTurns: Int = 0,
    regions: List<MapRegion> = emptyList(),
    selectedRegionId: String? = null,
    draftVertices: List<RegionPoint> = emptyList(),
    editingEnabled: Boolean = false,
    showNavPath: Boolean = false,
    onMapTap: (RegionPoint) -> Unit = {},
    modifier: Modifier = Modifier
) {
    var scale by remember { mutableStateOf(1f) }
    var pan by remember { mutableStateOf(Offset.Zero) }
    val bitmap = mapBuffer.bitmap
    val info = mapBuffer.info
    val pose = mapBuffer.pose
    val navPath = mapBuffer.navPath
    val imageVersion = mapBuffer.imageVersion
    val normalizedRotation = ((rotationTurns % 4) + 4) % 4

    LaunchedEffect(resetToken) {
        scale = 1f
        pan = Offset.Zero
    }

    LaunchedEffect(normalizedRotation) {
        pan = Offset.Zero
    }

    Box(
        modifier = modifier
            .clip(RoundedCornerShape(8.dp))
            .background(Color(0xFF1B2028))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(8.dp))
            .testTag("k1-map-canvas")
    ) {
    Canvas(
        modifier = Modifier
            .fillMaxSize()
            .pointerInput(editingEnabled, bitmap, info, scale, pan, normalizedRotation) {
                detectTapGestures { tap ->
                    val currentBitmap = bitmap ?: return@detectTapGestures
                    val currentInfo = info ?: return@detectTapGestures
                    if (!editingEnabled) {
                        return@detectTapGestures
                    }
                    val viewport = computeMapViewport(
                        canvasWidth = size.width.toFloat(),
                        canvasHeight = size.height.toFloat(),
                        bitmapWidth = currentBitmap.width.toFloat(),
                        bitmapHeight = currentBitmap.height.toFloat(),
                        rotationTurns = normalizedRotation,
                        scale = scale,
                        pan = pan
                    )
                    val rotatedX = (tap.x - viewport.topLeft.x) / viewport.drawScale
                    val rotatedY = (tap.y - viewport.topLeft.y) / viewport.drawScale
                    val mapped = rotatedToBitmapPoint(
                        rotatedX,
                        rotatedY,
                        currentBitmap.width.toFloat(),
                        currentBitmap.height.toFloat(),
                        normalizedRotation
                    )
                    val bitmapX = mapped.x
                    val bitmapY = mapped.y
                    if (bitmapX >= 0f && bitmapX < currentBitmap.width &&
                        bitmapY >= 0f && bitmapY < currentBitmap.height
                    ) {
                        onMapTap(
                            RegionPoint(
                                x = currentInfo.originX + bitmapX * currentInfo.resolution,
                                y = currentInfo.originY + (currentBitmap.height - 1f - bitmapY) * currentInfo.resolution
                            )
                        )
                    }
                }
            }
            .pointerInput(Unit) {
                detectTransformGestures { _, panChange, zoom, _ ->
                    scale = (scale * zoom).coerceIn(0.25f, 16f)
                    pan += panChange
                }
            }
    ) {
        if (bitmap == null || info == null) {
            drawCircle(Color(0xFF36404D), radius = min(size.width, size.height) * 0.18f)
            return@Canvas
        }

        @Suppress("UNUSED_VARIABLE")
        val imageVersionForRecompose = imageVersion
        val image = bitmap.asImageBitmap()
        val viewport = computeMapViewport(
            canvasWidth = size.width,
            canvasHeight = size.height,
            bitmapWidth = bitmap.width.toFloat(),
            bitmapHeight = bitmap.height.toFloat(),
            rotationTurns = normalizedRotation,
            scale = scale,
            pan = pan
        )

        withTransform({
            translate(viewport.topLeft.x, viewport.topLeft.y)
            scale(viewport.drawScale, viewport.drawScale, pivot = Offset.Zero)
            applyBitmapRotation(bitmap.width.toFloat(), bitmap.height.toFloat(), normalizedRotation)
        }) {
            drawImage(image)
            if (showNavPath && navPath != null && navPath.points.size >= 2) {
                val pathPoints = navPath.points.map { point ->
                    mapPointToBitmap(RegionPoint(point.x, point.y), info, bitmap.height)
                }
                for (i in 0 until pathPoints.lastIndex) {
                    drawLine(
                        color = Color(0xFF00FF00),
                        start = pathPoints[i],
                        end = pathPoints[i + 1],
                        strokeWidth = 1.6f,
                        cap = StrokeCap.Round
                    )
                }
                pathPoints.forEach { point ->
                    drawCircle(Color(0xFF00FF00), radius = 1.6f, center = point)
                }
            }
            regions.forEach { region ->
                drawRegionPolygon(
                    info = info,
                    mapHeight = bitmap.height,
                    vertices = region.vertices,
                    color = parseRegionColor(region.color),
                    selected = region.id == selectedRegionId
                )
                val labelCenter = region.center ?: if (closeable(region.vertices)) {
                    polygonCentroid(region.vertices)
                } else {
                    null
                }
                labelCenter?.let { center ->
                    val c = mapPointToBitmap(center, info, bitmap.height)
                    drawCircle(Color.White, radius = 4.0f, center = c)
                    drawCircle(parseRegionColor(region.color), radius = 2.3f, center = c)
                }
            }
            if (draftVertices.isNotEmpty()) {
                drawRegionPolygon(
                    info = info,
                    mapHeight = bitmap.height,
                    vertices = draftVertices,
                    color = Color(0xFFFFD166),
                    selected = true,
                    closed = false
                )
            }
            if (pose != null) {
                val px = (pose.x - info.originX) / info.resolution
                val py = bitmap.height - 1f - ((pose.y - info.originY) / info.resolution)
                val heading = pose.yaw - info.originYaw
                val dx = cos(heading)
                val dy = -sin(heading)
                val sideX = -dy
                val sideY = dx
                val front = Offset(
                    x = px + dx * 10f,
                    y = py + dy * 10f
                )
                val rearCenter = Offset(
                    x = px - dx * 5.5f,
                    y = py - dy * 5.5f
                )
                val leftRear = Offset(
                    x = rearCenter.x + sideX * 4.5f,
                    y = rearCenter.y + sideY * 4.5f
                )
                val rightRear = Offset(
                    x = rearCenter.x - sideX * 4.5f,
                    y = rearCenter.y - sideY * 4.5f
                )
                val robotShape = Path().apply {
                    moveTo(front.x, front.y)
                    lineTo(leftRear.x, leftRear.y)
                    lineTo(rightRear.x, rightRear.y)
                    close()
                }
                drawPath(
                    path = robotShape,
                    color = Color(0xFF4F8CFF)
                )
                drawPath(
                    path = robotShape,
                    color = Color.White,
                    style = Stroke(width = 2f)
                )
            }
        }
        regions.forEach { region ->
            val labelCenter = region.center ?: if (closeable(region.vertices)) {
                polygonCentroid(region.vertices)
            } else {
                null
            }
            labelCenter?.let { center ->
                val bitmapPoint = mapPointToBitmap(center, info, bitmap.height)
                val labelPoint = mapPointToCanvas(
                    bitmapPoint = bitmapPoint,
                    bitmapWidth = bitmap.width.toFloat(),
                    bitmapHeight = bitmap.height.toFloat(),
                    rotationTurns = normalizedRotation,
                    viewport = viewport
                )
                drawCenterLabel(region.name, labelPoint)
            }
        }
    }
        if (isLastMap && bitmap != null && info != null) {
            Box(
                modifier = Modifier
                    .align(Alignment.TopStart)
                    .padding(10.dp)
                    .clip(RoundedCornerShape(50))
                    .background(Color(0xCC202733))
                    .border(1.dp, Color(0xFFE6A23A), RoundedCornerShape(50))
                    .padding(horizontal = 10.dp, vertical = 5.dp)
            ) {
                Text(
                    text = "LAST MAP",
                    color = Color(0xFFFFD166),
                    fontSize = 9.sp,
                    lineHeight = 10.sp,
                    fontWeight = FontWeight.Black
                )
            }
        }
    }
}

internal data class MapViewport(
    val drawScale: Float,
    val topLeft: Offset,
    val rotatedWidth: Float,
    val rotatedHeight: Float
)

internal fun computeMapViewport(
    canvasWidth: Float,
    canvasHeight: Float,
    bitmapWidth: Float,
    bitmapHeight: Float,
    rotationTurns: Int,
    scale: Float,
    pan: Offset
): MapViewport {
    val safeBitmapWidth = max(1f, bitmapWidth)
    val safeBitmapHeight = max(1f, bitmapHeight)
    val normalizedRotation = ((rotationTurns % 4) + 4) % 4
    val rotatedWidth = if (normalizedRotation % 2 == 0) safeBitmapWidth else safeBitmapHeight
    val rotatedHeight = if (normalizedRotation % 2 == 0) safeBitmapHeight else safeBitmapWidth
    val baseFit = min(
        min(canvasWidth / safeBitmapWidth, canvasHeight / safeBitmapHeight),
        min(canvasWidth / safeBitmapHeight, canvasHeight / safeBitmapWidth)
    )
    val drawScale = max(0.001f, baseFit * scale)
    return MapViewport(
        drawScale = drawScale,
        topLeft = Offset(
            x = canvasWidth / 2f - rotatedWidth * drawScale / 2f + pan.x,
            y = canvasHeight / 2f - rotatedHeight * drawScale / 2f + pan.y
        ),
        rotatedWidth = rotatedWidth,
        rotatedHeight = rotatedHeight
    )
}

private fun DrawScope.drawRegionPolygon(
    info: K1MapInfo,
    mapHeight: Int,
    vertices: List<RegionPoint>,
    color: Color,
    selected: Boolean,
    closed: Boolean = true
) {
    if (vertices.isEmpty()) {
        return
    }
    val points = vertices.map { mapPointToBitmap(it, info, mapHeight) }
    val width = if (selected) 4f else 2.5f
    for (i in 0 until points.lastIndex) {
        drawLine(color, points[i], points[i + 1], strokeWidth = width)
    }
    if (closed && points.size >= 3) {
        drawLine(color, points.last(), points.first(), strokeWidth = width)
    }
    points.forEach { point ->
        drawCircle(color, radius = if (selected) 4.5f else 3.2f, center = point)
        drawCircle(Color.White, radius = if (selected) 5.6f else 4.0f, center = point, style = Stroke(width = 1.0f))
    }
}

private fun DrawScope.drawCenterLabel(name: String, center: Offset) {
    if (name.isBlank()) {
        return
    }
    val strokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = android.graphics.Color.WHITE
        textSize = 18f
        textAlign = Paint.Align.CENTER
        typeface = android.graphics.Typeface.create("sans-serif-black", android.graphics.Typeface.BOLD)
        style = Paint.Style.STROKE
        strokeWidth = 3f
    }
    val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = android.graphics.Color.BLACK
        textSize = 18f
        textAlign = Paint.Align.CENTER
        typeface = android.graphics.Typeface.create("sans-serif-black", android.graphics.Typeface.BOLD)
        style = Paint.Style.FILL
    }
    val y = center.y - 8f
    drawContext.canvas.nativeCanvas.drawText(name, center.x, y, strokePaint)
    drawContext.canvas.nativeCanvas.drawText(name, center.x, y, fillPaint)
}

internal fun mapPointToCanvas(
    bitmapPoint: Offset,
    bitmapWidth: Float,
    bitmapHeight: Float,
    rotationTurns: Int,
    viewport: MapViewport
): Offset {
    val rotatedPoint = bitmapToRotatedPoint(
        bitmapPoint.x,
        bitmapPoint.y,
        bitmapWidth,
        bitmapHeight,
        rotationTurns
    )
    return Offset(
        x = viewport.topLeft.x + rotatedPoint.x * viewport.drawScale,
        y = viewport.topLeft.y + rotatedPoint.y * viewport.drawScale
    )
}

private fun DrawTransform.applyBitmapRotation(width: Float, height: Float, turns: Int) {
    when (turns and 3) {
        1 -> {
            translate(left = height, top = 0f)
            rotate(degrees = 90f, pivot = Offset.Zero)
        }
        2 -> {
            translate(left = width, top = height)
            rotate(degrees = 180f, pivot = Offset.Zero)
        }
        3 -> {
            translate(left = 0f, top = width)
            rotate(degrees = -90f, pivot = Offset.Zero)
        }
    }
}

private fun rotatedToBitmapPoint(x: Float, y: Float, width: Float, height: Float, turns: Int): Offset {
    return when (turns and 3) {
        1 -> Offset(
            x = y.coerceIn(0f, width - 0.001f),
            y = (height - x).coerceIn(0f, height - 0.001f)
        )
        2 -> Offset(
            x = (width - x).coerceIn(0f, width - 0.001f),
            y = (height - y).coerceIn(0f, height - 0.001f)
        )
        3 -> Offset(
            x = (width - y).coerceIn(0f, width - 0.001f),
            y = x.coerceIn(0f, height - 0.001f)
        )
        else -> Offset(x, y)
    }
}

private fun bitmapToRotatedPoint(x: Float, y: Float, width: Float, height: Float, turns: Int): Offset {
    return when (turns and 3) {
        1 -> Offset(
            x = height - y,
            y = x
        )
        2 -> Offset(
            x = width - x,
            y = height - y
        )
        3 -> Offset(
            x = y,
            y = width - x
        )
        else -> Offset(x, y)
    }
}

private fun mapPointToBitmap(point: RegionPoint, info: K1MapInfo, mapHeight: Int): Offset {
    return Offset(
        x = (point.x - info.originX) / info.resolution,
        y = mapHeight - 1f - ((point.y - info.originY) / info.resolution)
    )
}

private fun parseRegionColor(value: String): Color {
    return try {
        Color(android.graphics.Color.parseColor(value))
    } catch (_: IllegalArgumentException) {
        Color(0xFF4F8CFF)
    }
}

@Composable
private fun K1LifecycleControls(
    connected: Boolean,
    endpointKind: K1EndpointKind,
    bridgeState: K1BridgeState,
    mapState: K1MapState,
    bridgeBusy: Boolean,
    mapBusy: Boolean,
    lastMapBase: String,
    mapSummary: String,
    poseSummary: String,
    manualEnabled: Boolean,
    onStartBridge: () -> Unit,
    onStopBridge: () -> Unit,
    onStartMapping: () -> Unit,
    onSaveMapping: () -> Unit,
    onStopMapping: () -> Unit,
    panelWidth: Dp,
    modifier: Modifier = Modifier
) {
    val onlineBridge = connected && endpointKind == K1EndpointKind.BRIDGE
    val supervisor = connected && endpointKind == K1EndpointKind.SUPERVISOR
    val active = bridgeBusy || mapBusy
    Column(
        modifier = modifier
            .width(panelWidth)
            .fillMaxHeight()
            .clip(RoundedCornerShape(7.dp))
            .background(Color(0xE6111820))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(7.dp))
            .padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Text(
            text = "K1 CONTROL",
            color = Color(0xFFB3C6DE),
            fontSize = 8.sp,
            lineHeight = 9.sp,
            fontWeight = FontWeight.ExtraBold
        )
        Text(
            text = when (endpointKind) {
                K1EndpointKind.SUPERVISOR -> "SUPERVISOR"
                K1EndpointKind.BRIDGE -> "BRIDGE ${bridgeState.name}"
                K1EndpointKind.UNKNOWN -> if (connected) "DETECTING" else "OFFLINE"
            },
            color = Color.White,
            fontSize = 14.sp,
            lineHeight = 16.sp,
            fontWeight = FontWeight.Black,
            maxLines = 1,
            overflow = TextOverflow.Clip
        )
        Text(
            text = "MAP ${mapState.name}",
            color = Color(0xFFD6E1EF),
            fontSize = 10.sp,
            lineHeight = 12.sp,
            maxLines = 2,
            overflow = TextOverflow.Clip
        )

        K1PanelField("MAP", mapSummary)
        K1PanelField("POSE", poseSummary)
        K1PanelField("MANUAL", if (manualEnabled) "ON" else "OFF")

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            K1LifecycleButton(
                text = "Start Bridge",
                enabled = supervisor && !bridgeBusy,
                color = Color(0xFF20A66B),
                onClick = onStartBridge,
                modifier = Modifier.weight(1f),
                holdColorWhenDisabled = supervisor && bridgeBusy
            )
            K1LifecycleButton(
                text = "Start Map",
                enabled = onlineBridge && !active && mapState != K1MapState.MAPPING,
                color = Color(0xFF4F8CFF),
                onClick = onStartMapping,
                modifier = Modifier.weight(1f),
                holdColorWhenDisabled = mapBusy && mapState == K1MapState.STARTING
            )
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            K1LifecycleButton(
                text = "Save Map",
                enabled = onlineBridge && !active && mapState == K1MapState.MAPPING,
                color = Color(0xFFE6A23A),
                onClick = onSaveMapping,
                modifier = Modifier.weight(1f),
                holdColorWhenDisabled = mapBusy && mapState == K1MapState.SAVING
            )
            K1LifecycleButton(
                text = "Stop Map",
                enabled = onlineBridge && !active && mapState == K1MapState.MAPPING,
                color = Color(0xFFE53935),
                onClick = onStopMapping,
                modifier = Modifier.weight(1f),
                holdColorWhenDisabled = mapBusy && mapState == K1MapState.STOPPING
            )
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            K1LifecycleButton(
                text = "Stop Bridge",
                enabled = onlineBridge && !bridgeBusy && !mapBusy && mapState == K1MapState.IDLE,
                color = Color(0xFF7D5FFF),
                onClick = onStopBridge,
                modifier = Modifier.fillMaxWidth(),
                holdColorWhenDisabled = onlineBridge && bridgeBusy
            )
        }
    }
}

@Composable
private fun K1PanelField(
    label: String,
    value: String,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .height(42.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF1B2028))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(6.dp))
            .padding(horizontal = 8.dp, vertical = 5.dp),
        verticalArrangement = Arrangement.Center
    ) {
        Text(
            text = label,
            color = Color(0xFFB3C6DE),
            fontSize = 7.sp,
            lineHeight = 8.sp,
            fontWeight = FontWeight.ExtraBold,
            maxLines = 1
        )
        Text(
            text = value,
            color = Color.White,
            fontSize = 10.sp,
            lineHeight = 11.sp,
            fontWeight = FontWeight.Black,
            maxLines = 2,
            overflow = TextOverflow.Clip
        )
    }
}

@Composable
private fun K1LifecycleButton(
    text: String,
    enabled: Boolean,
    color: Color,
    onClick: () -> Unit,
    modifier: Modifier = Modifier,
    holdColorWhenDisabled: Boolean = false
) {
    Button(
        onClick = onClick,
        enabled = enabled,
        modifier = modifier
            .fillMaxWidth()
            .height(34.dp),
        contentPadding = PaddingValues(0.dp),
        colors = ButtonDefaults.buttonColors(
            containerColor = color,
            contentColor = Color.White,
            disabledContainerColor = if (holdColorWhenDisabled) color else Color(0xFF3A414D),
            disabledContentColor = if (holdColorWhenDisabled) Color.White else Color(0xFFB9C6D6)
        ),
        shape = RoundedCornerShape(6.dp)
    ) {
        Text(
            text = text,
            fontSize = 8.sp,
            lineHeight = 9.sp,
            fontWeight = FontWeight.Black,
            textAlign = TextAlign.Center
        )
    }
}

@Composable
private fun K1ManualControls(
    connected: Boolean,
    manualEnabled: Boolean,
    speedScale: Float,
    omegaScale: Float,
    vx: Float,
    vy: Float,
    omega: Float,
    onManualToggle: () -> Unit,
    onSpeedScaleChange: (Float) -> Unit,
    onOmegaScaleChange: (Float) -> Unit,
    onJoystickChange: (Float, Float) -> Unit,
    onOmegaChange: (Float) -> Unit,
    modifier: Modifier = Modifier
) {
    val blue = Color(0xFF4F8CFF)
    val green = Color(0xFF20A66B)
    val red = Color(0xFFE53935)

    Row(
        modifier = modifier
            .clip(RoundedCornerShape(7.dp))
            .background(Color(0xE6111820))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(7.dp))
            .padding(8.dp),
        horizontalArrangement = Arrangement.spacedBy(10.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Column(
            modifier = Modifier.width(292.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Row(
                horizontalArrangement = Arrangement.spacedBy(8.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Button(
                    onClick = onManualToggle,
                    enabled = connected,
                    modifier = Modifier
                        .width(68.dp)
                        .height(42.dp),
                    contentPadding = PaddingValues(0.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (manualEnabled) red else green,
                        contentColor = Color.White,
                        disabledContainerColor = Color(0xFF3A414D),
                        disabledContentColor = Color(0xFFB9C6D6)
                    ),
                    shape = RoundedCornerShape(6.dp)
                ) {
                    Text(
                        text = if (manualEnabled) "STOP" else "ENABLE",
                        fontSize = 8.sp,
                        lineHeight = 9.sp,
                        fontWeight = FontWeight.Black,
                        textAlign = TextAlign.Center
                    )
                }
                K1ScalePanel("SPEED", speedScale, blue, Modifier.width(104.dp), onSpeedScaleChange)
                K1ScalePanel("ANGULAR", omegaScale, blue, Modifier.width(104.dp), onOmegaScaleChange)
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                K1Metric("VX", vx.format2(), Modifier.width(68.dp))
                K1Metric("VY", vy.format2(), Modifier.width(68.dp))
                K1Metric("W", omega.format2(), Modifier.width(68.dp))
            }
        }
        K1JoystickControl(
            speedScale = speedScale,
            enabled = connected && manualEnabled,
            onChange = onJoystickChange,
            modifier = Modifier.size(104.dp)
        )
        K1OmegaSlider(
            omegaScale = omegaScale,
            enabled = connected && manualEnabled,
            onChange = onOmegaChange,
            modifier = Modifier
                .weight(1f)
                .height(54.dp)
        )
    }
}

@Composable
private fun K1Metric(label: String, value: String, modifier: Modifier = Modifier) {
    Column(
        modifier = modifier
            .height(42.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF232A34))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(6.dp))
            .padding(horizontal = 6.dp, vertical = 4.dp),
        verticalArrangement = Arrangement.Center
    ) {
        Text(
            label,
            color = Color(0xFFB3C6DE),
            fontSize = 7.sp,
            lineHeight = 8.sp,
            fontWeight = FontWeight.ExtraBold
        )
        Spacer(Modifier.height(3.dp))
        Text(
            value,
            color = Color.White,
            fontSize = 9.sp,
            lineHeight = 9.sp,
            fontWeight = FontWeight.Black,
            maxLines = 2,
            overflow = TextOverflow.Clip
        )
    }
}

@Composable
private fun K1ScalePanel(
    title: String,
    selected: Float,
    activeColor: Color,
    modifier: Modifier,
    onChange: (Float) -> Unit
) {
    Column(
        modifier = modifier
            .height(42.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF1B2028))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(6.dp))
            .padding(5.dp)
    ) {
        Text(
            text = title,
            color = Color(0xFFB3C6DE),
            fontSize = 7.sp,
            lineHeight = 8.sp,
            fontWeight = FontWeight.ExtraBold
        )
        Spacer(Modifier.height(3.dp))
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(2.dp)
        ) {
            listOf(0.25f, 0.5f, 0.75f, 1.0f).forEach { value ->
                Box(
                    modifier = Modifier
                        .weight(1f)
                        .height(18.dp)
                        .clip(RoundedCornerShape(5.dp))
                        .background(if (selected == value) activeColor else Color(0xFF232A34))
                        .border(1.dp, Color(0xFF36404D), RoundedCornerShape(5.dp))
                        .clickable { onChange(value) },
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = if (value == 1.0f) "1x" else (value * 100).toInt().toString(),
                        color = Color.White,
                        fontSize = 7.sp,
                        fontWeight = FontWeight.Black
                    )
                }
            }
        }
    }
}

@Composable
private fun K1JoystickControl(
    speedScale: Float,
    enabled: Boolean,
    onChange: (Float, Float) -> Unit,
    modifier: Modifier = Modifier
) {
    val density = LocalDensity.current
    var sizePx by remember { mutableIntStateOf(1) }
    var thumbOffset by remember { mutableStateOf(Offset.Zero) }
    val thumbSizePx = with(density) { 48.dp.toPx() }

    fun update(position: Offset) {
        if (!enabled) {
            return
        }
        val center = Offset(sizePx / 2f, sizePx / 2f)
        val dx = position.x - center.x
        val dy = center.y - position.y
        val maxRadius = max(1f, sizePx / 2f - thumbSizePx / 2f)
        val distance = hypot(dx, dy).coerceAtMost(maxRadius)
        val angle = atan2(dy, dx)
        thumbOffset = Offset(
            x = cos(angle) * distance,
            y = -sin(angle) * distance
        )
        val normalizedX = thumbOffset.x / maxRadius
        val normalizedY = -thumbOffset.y / maxRadius
        onChange(
            normalizedY * K1_MAX_LINEAR_MPS * speedScale,
            -normalizedX * K1_MAX_LINEAR_MPS * speedScale
        )
    }

    fun reset() {
        thumbOffset = Offset.Zero
        onChange(0f, 0f)
    }

    LaunchedEffect(enabled) {
        if (!enabled) {
            reset()
        }
    }

    Box(
        modifier = modifier
            .pointerInput(enabled, speedScale) {
                detectDragGestures(
                    onDragStart = { update(it) },
                    onDrag = { change, _ ->
                        update(change.position)
                        change.consume()
                    },
                    onDragEnd = ::reset,
                    onDragCancel = ::reset
                )
            }
            .onSizeChanged { sizePx = it.width },
        contentAlignment = Alignment.Center
    ) {
        Canvas(Modifier.fillMaxSize()) {
            val radius = size.minDimension / 2f
            val center = Offset(size.width / 2f, size.height / 2f)
            drawCircle(Color(0xFF202733), radius)
            drawCircle(Color(0xFF4F5B6B), radius, style = Stroke(width = 3.dp.toPx()))
            drawCircle(Color(0x334F8CFF), radius * 0.24f)
            drawCircle(Color(0x26FFFFFF), radius * 0.62f, style = Stroke(width = 1.dp.toPx()))
            drawCircle(Color(0x22FFFFFF), radius * 0.35f, style = Stroke(width = 1.dp.toPx()))
            drawLine(
                Color(0x33FFFFFF),
                Offset(center.x, 0f),
                Offset(center.x, size.height),
                strokeWidth = 1.dp.toPx()
            )
            drawLine(
                Color(0x33FFFFFF),
                Offset(0f, center.y),
                Offset(size.width, center.y),
                strokeWidth = 1.dp.toPx()
            )
        }

        Box(
            modifier = Modifier
                .offset {
                    IntOffset(
                        thumbOffset.x.roundToInt(),
                        thumbOffset.y.roundToInt()
                    )
                }
                .size(48.dp)
                .clip(CircleShape)
                .background(if (enabled) Color(0xFF4F8CFF) else Color(0xFF647084))
                .border(3.dp, Color(0xFFEFF4FF), CircleShape)
        )
    }
}

@Composable
private fun K1OmegaSlider(
    omegaScale: Float,
    enabled: Boolean,
    onChange: (Float) -> Unit,
    modifier: Modifier = Modifier
) {
    var widthPx by remember { mutableIntStateOf(1) }
    var thumbX by remember { mutableFloatStateOf(0f) }
    val density = LocalDensity.current
    val thumbHalfWidth = with(density) { 26.dp.toPx() }

    fun setFromPosition(x: Float) {
        if (!enabled) {
            return
        }
        val half = max(1f, widthPx / 2f - thumbHalfWidth)
        val clamped = (x - widthPx / 2f).coerceIn(-half, half)
        val normalized = clamped / half
        thumbX = clamped
        onChange(-normalized * K1_MAX_ANGULAR_RADPS * omegaScale)
    }

    fun reset() {
        thumbX = 0f
        onChange(0f)
    }

    LaunchedEffect(enabled) {
        if (!enabled) {
            reset()
        }
    }

    Column(
        modifier = modifier
            .clip(RoundedCornerShape(7.dp))
            .background(Color(0xFF1B2028))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(7.dp))
            .padding(4.dp)
    ) {
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(34.dp)
                .clip(RoundedCornerShape(50))
                .background(
                    Brush.horizontalGradient(
                        listOf(
                            Color(0x8058401F),
                            Color(0x30232A34),
                            Color(0x8058401F)
                        )
                    )
                )
                .border(1.dp, Color(0xFF36404D), RoundedCornerShape(50))
                .onSizeChanged { widthPx = it.width }
                .pointerInput(enabled, omegaScale) {
                    detectDragGestures(
                        onDragStart = { setFromPosition(it.x) },
                        onDrag = { change, _ ->
                            setFromPosition(change.position.x)
                            change.consume()
                        },
                        onDragEnd = ::reset,
                        onDragCancel = ::reset
                    )
                },
            contentAlignment = Alignment.Center
        ) {
            Canvas(Modifier.fillMaxSize()) {
                drawLine(
                    Color(0x80FFFFFF),
                    Offset(size.width / 2f, 8.dp.toPx()),
                    Offset(size.width / 2f, size.height - 8.dp.toPx()),
                    strokeWidth = 2.dp.toPx(),
                    cap = StrokeCap.Round
                )
            }
            Box(
                modifier = Modifier
                    .offset { IntOffset(thumbX.roundToInt(), 0) }
                    .width(46.dp)
                    .height(26.dp)
                    .clip(RoundedCornerShape(50))
                    .background(if (enabled) Color(0xFFE6A23A) else Color(0xFF756246))
                    .border(3.dp, Color.White, RoundedCornerShape(50))
            )
        }
    }
}

@Composable
private fun K1DeviceSelector(
    devices: List<BondedDeviceInfo>,
    selectedAddress: String?,
    selectedName: String,
    connected: Boolean,
    connecting: Boolean,
    hasBluetoothPermission: Boolean,
    onRequestPermission: () -> Unit,
    onRefreshDevices: () -> Unit,
    onSelectDevice: (BondedDeviceInfo) -> Unit,
    onConnectToggle: () -> Unit,
    modifier: Modifier = Modifier
) {
    var menuExpanded by remember { mutableStateOf(false) }

    Row(
        modifier = modifier
            .height(34.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF1B2028))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(6.dp))
            .padding(3.dp),
        horizontalArrangement = Arrangement.spacedBy(4.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Box(Modifier.weight(1f)) {
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(26.dp)
                    .clip(RoundedCornerShape(50))
                    .background(Color(0xFF232A34))
                    .border(1.dp, Color(0xFF36404D), RoundedCornerShape(50))
                    .clickable {
                        if (hasBluetoothPermission) {
                            onRefreshDevices()
                            menuExpanded = true
                        } else {
                            onRequestPermission()
                        }
                    },
                contentAlignment = Alignment.Center
            ) {
                Text(
                    selectedName,
                    fontSize = 9.sp,
                    color = Color.White,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis,
                    textAlign = TextAlign.Center,
                    modifier = Modifier.fillMaxWidth()
                )
            }
            DropdownMenu(expanded = menuExpanded, onDismissRequest = { menuExpanded = false }) {
                if (devices.isEmpty()) {
                    DropdownMenuItem(
                        text = { Text("No bonded devices") },
                        onClick = { menuExpanded = false }
                    )
                }
                devices.forEach { device ->
                    DropdownMenuItem(
                        text = { Text("${device.name} ${device.address}") },
                        onClick = {
                            onSelectDevice(device)
                            menuExpanded = false
                        }
                    )
                }
            }
        }
        Button(
            onClick = onConnectToggle,
            enabled = selectedAddress != null || connected || connecting,
            modifier = Modifier
                .width(50.dp)
                .height(26.dp),
            contentPadding = PaddingValues(0.dp),
            colors = ButtonDefaults.buttonColors(
                containerColor = Color(0xFF20304D),
                contentColor = Color.White
            )
        ) {
            Text(
                when {
                    connecting -> "..."
                    connected -> "Off"
                    else -> "Link"
                },
                fontSize = 9.sp,
                fontWeight = FontWeight.Bold
            )
        }
    }
}

@Composable
private fun StatusChip(label: String, value: String, modifier: Modifier = Modifier.width(70.dp)) {
    Column(
        modifier = modifier
            .height(34.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF232A34))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(6.dp))
            .padding(horizontal = 6.dp, vertical = 3.dp),
        verticalArrangement = Arrangement.Center
    ) {
        Text(
            label,
            color = Color(0xFFB3C6DE),
            fontSize = 6.sp,
            lineHeight = 7.sp,
            fontWeight = FontWeight.ExtraBold
        )
        Spacer(Modifier.height(2.dp))
        Text(
            value,
            color = Color.White,
            fontSize = 8.sp,
            lineHeight = 8.sp,
            fontWeight = FontWeight.Black,
            maxLines = 2,
            overflow = TextOverflow.Clip
        )
    }
}

@Composable
fun ModeSelectionScreen(onDirect: () -> Unit, onK1Map: () -> Unit) {
    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFF0F1217))
            .padding(28.dp),
        contentAlignment = Alignment.Center
    ) {
        Row(horizontalArrangement = Arrangement.spacedBy(18.dp)) {
            ModeCard(
                title = "Direct SPP",
                subtitle = "Phone -> STM32 HC-05\nSimple chassis control",
                onClick = onDirect
            )
            ModeCard(
                title = "K1 Map/Nav",
                subtitle = "Phone -> K1 Bluetooth\nLive /map and robot pose",
                onClick = onK1Map
            )
        }
    }
}

@Composable
private fun ModeCard(title: String, subtitle: String, onClick: () -> Unit) {
    Column(
        modifier = Modifier
            .width(230.dp)
            .height(150.dp)
            .clip(RoundedCornerShape(8.dp))
            .background(Color(0xFF1B2028))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(8.dp))
            .clickable(onClick = onClick)
            .padding(18.dp),
        verticalArrangement = Arrangement.Center,
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(title, color = Color.White, fontSize = 24.sp, fontWeight = FontWeight.Black)
        Spacer(Modifier.height(14.dp))
        Text(
            subtitle,
            color = Color(0xFFB3C6DE),
            fontSize = 13.sp,
            lineHeight = 17.sp,
            textAlign = TextAlign.Center
        )
    }
}

private fun Float.format2(): String = "%.2f".format(this)

private const val K1_MAX_LINEAR_MPS = 1.4f
private const val K1_MAX_ANGULAR_RADPS = 3.7f
