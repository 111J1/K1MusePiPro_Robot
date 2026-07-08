package com.embodiedai.robotcontroller

import android.Manifest
import android.os.Build
import android.os.Bundle
import android.os.SystemClock
import android.view.View
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.onSizeChanged
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.zIndex
import com.embodiedai.robotcontroller.bluetooth.BluetoothSppClient
import com.embodiedai.robotcontroller.bluetooth.BondedDeviceInfo
import com.embodiedai.robotcontroller.bluetooth.K1MapBluetoothClient
import com.embodiedai.robotcontroller.protocol.ControlProtocol
import com.embodiedai.robotcontroller.protocol.CoordinateMode
import com.embodiedai.robotcontroller.ui.theme.RobotControllerTheme
import kotlinx.coroutines.delay
import java.time.LocalTime
import java.time.format.DateTimeFormatter
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.roundToInt
import kotlin.math.sin

private const val SEND_PERIOD_MS = 50L
private const val IDLE_STOP_PERIOD_MS = 100L
private const val IDLE_STOP_KEEPALIVE_MS = 1000L
private const val TWO_PI = (Math.PI * 2.0).toFloat()

private enum class AppMode {
    DIRECT_SPP,
    K1_MAP_NAV
}

class MainActivity : ComponentActivity() {
    private lateinit var bluetoothClient: BluetoothSppClient
    private lateinit var k1MapClient: K1MapBluetoothClient

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        hideSystemBars()

        bluetoothClient = BluetoothSppClient(applicationContext)
        k1MapClient = K1MapBluetoothClient(applicationContext)
        lateinit var refreshDevices: () -> Unit
        var updatePermissionState: ((Boolean) -> Unit)? = null
        val permissionLauncher = registerForActivityResult(
            ActivityResultContracts.RequestMultiplePermissions()
        ) { results ->
            val granted = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                results[Manifest.permission.BLUETOOTH_CONNECT] == true
            } else {
                true
            }
            updatePermissionState?.invoke(granted)
            if (granted) {
                refreshDevices()
            }
        }

        setContent {
            RobotControllerTheme(dynamicColor = false, darkTheme = true) {
                var appMode by remember { mutableStateOf<AppMode?>(null) }
                var hasBluetoothPermission by remember {
                    mutableStateOf(bluetoothClient.hasConnectPermission())
                }
                val devices = remember { mutableStateListOf<BondedDeviceInfo>() }
                var selectedAddress by remember { mutableStateOf<String?>(null) }
                var selectedName by remember { mutableStateOf("No device") }
                var connected by remember { mutableStateOf(false) }
                var connecting by remember { mutableStateOf(false) }
                var masterEnabled by remember { mutableStateOf(false) }
                var mode by remember { mutableStateOf(CoordinateMode.LCS) }
                var speedScale by remember { mutableFloatStateOf(0.5f) }
                var omegaScale by remember { mutableFloatStateOf(0.5f) }
                var direction by remember { mutableFloatStateOf(0f) }
                var velocity by remember { mutableFloatStateOf(0f) }
                var omega by remember { mutableFloatStateOf(0f) }
                var command by remember { mutableStateOf("STOP") }
                var seq by remember { mutableIntStateOf(0) }
                val logs = remember { mutableStateListOf("STOP frame ready.") }

                fun addLog(message: String) {
                    val time = LocalTime.now().format(DateTimeFormatter.ofPattern("HH:mm:ss"))
                    logs.add(0, "[$time] $message")
                    while (logs.size > 40) {
                        logs.removeAt(logs.lastIndex)
                    }
                }

                fun nextSeq(): Int {
                    seq = (seq + 1) and 0xff
                    return seq
                }

                fun markDisconnected() {
                    connected = false
                    connecting = false
                    masterEnabled = false
                    direction = 0f
                    velocity = 0f
                    omega = 0f
                    command = "STOP"
                    addLog("Bluetooth send failed; disconnected")
                }

                fun sendStop(log: Boolean = false, reason: String = "stop") {
                    if (!connected) {
                        command = "STOP"
                        return
                    }
                    val frameSeq = nextSeq()
                    command = "STOP"
                    bluetoothClient.sendAsync(ControlProtocol.stop(frameSeq), ::markDisconnected)
                    if (log) {
                        addLog("SEQ $frameSeq: STOP ($reason)")
                    }
                }

                fun sendMove(log: Boolean = false, reason: String = "move") {
                    if (!connected) {
                        return
                    }
                    val frameSeq = nextSeq()
                    command = "MOV"
                    bluetoothClient.sendAsync(
                        ControlProtocol.move(frameSeq, mode, direction, velocity, omega),
                        ::markDisconnected
                    )
                    if (log) {
                        addLog(
                            "SEQ $frameSeq: MOV ${mode.name} dir=${direction.format3()} " +
                                "v=${velocity.format3()} omega=${omega.format3()} ($reason)"
                        )
                    }
                }

                fun sendZeroWcsMove(reason: String) {
                    if (!connected || !masterEnabled) {
                        return
                    }
                    val frameSeq = nextSeq()
                    command = "MOV"
                    bluetoothClient.sendAsync(
                        ControlProtocol.move(
                            frameSeq,
                            CoordinateMode.WCS,
                            direction = 0f,
                            velocity = 0f,
                            omega = 0f
                        ),
                        ::markDisconnected
                    )
                    addLog("SEQ $frameSeq: MOV WCS dir=0 v=0 omega=0 ($reason)")
                }

                fun sendOdomReset() {
                    if (!connected || !masterEnabled) {
                        addLog("ODOM reset blocked")
                        return
                    }
                    val frameSeq = nextSeq()
                    bluetoothClient.sendAsync(
                        ControlProtocol.resetOdometry(frameSeq),
                        ::markDisconnected
                    )
                    addLog("SEQ $frameSeq: ODOM reset direction=0 x=0 y=0")
                }

                fun loadBondedDevices() {
                    devices.clear()
                    devices.addAll(bluetoothClient.bondedDevices())
                    if (selectedAddress == null && devices.isNotEmpty()) {
                        selectedAddress = devices.first().address
                        selectedName = devices.first().name
                    }
                }

                refreshDevices = ::loadBondedDevices
                updatePermissionState = { granted ->
                    hasBluetoothPermission = granted
                }

                LaunchedEffect(hasBluetoothPermission) {
                    if (hasBluetoothPermission) {
                        loadBondedDevices()
                    }
                }

                LaunchedEffect(Unit) {
                    if (!bluetoothClient.hasConnectPermission() &&
                        Build.VERSION.SDK_INT >= Build.VERSION_CODES.S
                    ) {
                        permissionLauncher.launch(
                            arrayOf(Manifest.permission.BLUETOOTH_CONNECT)
                        )
                    }
                }

                if (appMode == null) {
                    ModeSelectionScreen(
                        onDirect = {
                            k1MapClient.disconnect()
                            appMode = AppMode.DIRECT_SPP
                        },
                        onK1Map = {
                            if (connected || connecting) {
                                val stopFrame = if (connected) {
                                    ControlProtocol.stop((seq + 1) and 0xff)
                                } else {
                                    null
                                }
                                bluetoothClient.disconnect(stopFrame)
                            }
                            connected = false
                            connecting = false
                            masterEnabled = false
                            direction = 0f
                            velocity = 0f
                            omega = 0f
                            command = "STOP"
                            appMode = AppMode.K1_MAP_NAV
                        }
                    )
                    return@RobotControllerTheme
                }

                if (appMode == AppMode.K1_MAP_NAV) {
                    K1MapModeScreen(
                        bluetoothClient = k1MapClient,
                        hasBluetoothPermission = hasBluetoothPermission,
                        devices = devices,
                        onRequestPermission = {
                            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                                permissionLauncher.launch(
                                    arrayOf(Manifest.permission.BLUETOOTH_CONNECT)
                                )
                            }
                        },
                        onRefreshDevices = ::loadBondedDevices,
                        onBack = {
                            appMode = null
                        }
                    )
                    return@RobotControllerTheme
                }

                val latestConnected by rememberUpdatedState(connected)
                val latestMasterEnabled by rememberUpdatedState(masterEnabled)
                val latestVelocity by rememberUpdatedState(velocity)
                val latestOmega by rememberUpdatedState(omega)

                LaunchedEffect(Unit) {
                    var wasMoving = false
                    var lastIdleStopMs = 0L
                    while (true) {
                        if (latestConnected) {
                            val moving = latestVelocity > 0.001f ||
                                kotlin.math.abs(latestOmega) > 0.001f
                            if (latestMasterEnabled && moving) {
                                sendMove()
                                wasMoving = true
                                delay(SEND_PERIOD_MS)
                            } else {
                                val nowMs = SystemClock.uptimeMillis()
                                if (wasMoving || nowMs - lastIdleStopMs >= IDLE_STOP_KEEPALIVE_MS) {
                                    sendStop()
                                    lastIdleStopMs = nowMs
                                }
                                wasMoving = false
                                delay(IDLE_STOP_PERIOD_MS)
                            }
                        } else {
                            wasMoving = false
                            delay(200L)
                        }
                    }
                }

                DisposableEffect(Unit) {
                    onDispose {
                        bluetoothClient.disconnect(ControlProtocol.stop((seq + 1) and 0xff))
                    }
                }

                RobotControllerScreen(
                    hasBluetoothPermission = hasBluetoothPermission,
                    devices = devices,
                    selectedAddress = selectedAddress,
                    selectedName = selectedName,
                    connected = connected,
                    connecting = connecting,
                    masterEnabled = masterEnabled,
                    mode = mode,
                    speedScale = speedScale,
                    omegaScale = omegaScale,
                    direction = direction,
                    velocity = velocity,
                    omega = omega,
                    command = command,
                    seq = seq,
                    logs = logs,
                    onRequestPermission = {
                        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                            permissionLauncher.launch(
                                arrayOf(Manifest.permission.BLUETOOTH_CONNECT)
                            )
                        }
                    },
                    onRefreshDevices = ::loadBondedDevices,
                    onSelectDevice = { device ->
                        selectedAddress = device.address
                        selectedName = device.name
                    },
                    onConnectToggle = {
                        if (connected || connecting) {
                            val stopFrame = if (connected) {
                                val frameSeq = nextSeq()
                                command = "STOP"
                                addLog("SEQ $frameSeq: STOP (disconnect)")
                                ControlProtocol.stop(frameSeq)
                            } else {
                                null
                            }
                            bluetoothClient.disconnect(stopFrame) {
                                connected = false
                                connecting = false
                                masterEnabled = false
                                direction = 0f
                                velocity = 0f
                                omega = 0f
                                command = "STOP"
                                addLog("Disconnected")
                            }
                        } else {
                            val address = selectedAddress
                            if (address == null) {
                                addLog("Select a bonded device first")
                                return@RobotControllerScreen
                            }
                            connecting = true
                            bluetoothClient.connect(address) { success, message ->
                                connecting = false
                                connected = success
                                masterEnabled = false
                                command = "STOP"
                                addLog(message)
                            }
                        }
                    },
                    onMasterToggle = {
                        masterEnabled = !masterEnabled
                        if (!masterEnabled) {
                            velocity = 0f
                            omega = 0f
                            sendStop(log = true, reason = "master off")
                        } else {
                            sendStop(log = true, reason = "master on")
                        }
                    },
                    onModeChange = { newMode ->
                        if (mode == CoordinateMode.WCS && newMode != CoordinateMode.WCS) {
                            sendZeroWcsMove("leave WCS")
                        }
                        mode = newMode
                        addLog("Mode changed to ${newMode.name}")
                    },
                    onSpeedScaleChange = { speedScale = it },
                    onOmegaScaleChange = { omegaScale = it },
                    onJoystickChange = { newDirection, newVelocity ->
                        val wasMovingInWcs = mode == CoordinateMode.WCS && velocity > 0.001f
                        direction = newDirection
                        velocity = newVelocity
                        if (wasMovingInWcs && newVelocity <= 0.001f) {
                            sendZeroWcsMove("joystick centered")
                        }
                    },
                    onOmegaChange = { omega = it },
                    onOdomReset = ::sendOdomReset,
                    onBack = {
                        val stopFrame = if (connected) {
                            val frameSeq = nextSeq()
                            command = "STOP"
                            addLog("SEQ $frameSeq: STOP (back)")
                            ControlProtocol.stop(frameSeq)
                        } else {
                            null
                        }
                        bluetoothClient.disconnect(stopFrame) {
                            connected = false
                            connecting = false
                            masterEnabled = false
                            direction = 0f
                            velocity = 0f
                            omega = 0f
                            command = "STOP"
                            appMode = null
                        }
                    }
                )
            }
        }
    }

    override fun onStop() {
        super.onStop()
        disconnectForLifecycleExit()
    }

    override fun onDestroy() {
        disconnectForLifecycleExit()
        super.onDestroy()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) {
            hideSystemBars()
        }
    }

    private fun disconnectForLifecycleExit() {
        if (::bluetoothClient.isInitialized) {
            bluetoothClient.disconnect(ControlProtocol.stop(0))
        }
        if (::k1MapClient.isInitialized) {
            k1MapClient.disconnect()
        }
    }

    private fun hideSystemBars() {
        WindowCompat.setDecorFitsSystemWindows(window, false)
        window.decorView.systemUiVisibility = (
            View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                or View.SYSTEM_UI_FLAG_FULLSCREEN
                or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                or View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                or View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                or View.SYSTEM_UI_FLAG_LAYOUT_STABLE
            )
        WindowInsetsControllerCompat(window, window.decorView).apply {
            hide(WindowInsetsCompat.Type.systemBars())
            systemBarsBehavior =
                WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }
    }
}

@Composable
private fun RobotControllerScreen(
    hasBluetoothPermission: Boolean,
    devices: List<BondedDeviceInfo>,
    selectedAddress: String?,
    selectedName: String,
    connected: Boolean,
    connecting: Boolean,
    masterEnabled: Boolean,
    mode: CoordinateMode,
    speedScale: Float,
    omegaScale: Float,
    direction: Float,
    velocity: Float,
    omega: Float,
    command: String,
    seq: Int,
    logs: List<String>,
    onRequestPermission: () -> Unit,
    onRefreshDevices: () -> Unit,
    onSelectDevice: (BondedDeviceInfo) -> Unit,
    onConnectToggle: () -> Unit,
    onMasterToggle: () -> Unit,
    onModeChange: (CoordinateMode) -> Unit,
    onSpeedScaleChange: (Float) -> Unit,
    onOmegaScaleChange: (Float) -> Unit,
    onJoystickChange: (Float, Float) -> Unit,
    onOmegaChange: (Float) -> Unit,
    onOdomReset: () -> Unit,
    onBack: () -> Unit
) {
    val bg = Color(0xFF0F1217)
    val panel2 = Color(0xFF232A34)
    val line = Color(0xFF36404D)
    val blue = Color(0xFF4F8CFF)
    val red = Color(0xFFE53935)
    val green = Color(0xFF20A66B)
    var controlsExpanded by remember { mutableStateOf(false) }
    var logExpanded by remember { mutableStateOf(false) }

    BoxWithConstraints(
        modifier = Modifier
            .fillMaxSize()
            .background(bg)
            .padding(6.dp)
    ) {
        Row(
            modifier = Modifier
                .align(Alignment.TopStart)
                .offset(x = 314.dp)
                .width(maxWidth - 320.dp)
                .height(42.dp)
                .padding(end = 6.dp),
            horizontalArrangement = Arrangement.spacedBy(5.dp)
        ) {
            CommandMetric("COMMAND", command, command == "STOP", Modifier.weight(0.85f))
            CommandMetric("MODE", mode.name, false, Modifier.weight(0.62f))
            CommandMetric("DIR", direction.formatDegrees(), false, Modifier.weight(0.68f))
            CommandMetric("VEL", velocity.format3(), false, Modifier.weight(0.74f))
            CommandMetric("OMEGA", omega.format3(), false, Modifier.weight(0.78f))
            CommandMetric("SEQ", seq.toString().padStart(3, '0'), false, Modifier.weight(0.58f))
            Box(
                modifier = Modifier
                    .weight(2.2f)
                    .height(42.dp)
                    .clip(RoundedCornerShape(5.dp))
                    .background(Color(0xFF111820))
                    .border(1.dp, line, RoundedCornerShape(5.dp))
                    .clickable { logExpanded = true }
                    .padding(horizontal = 8.dp, vertical = 3.dp)
            ) {
                Text(
                    text = logs.joinToString("\n"),
                    color = Color(0xFFD6E1EF),
                    fontSize = 8.sp,
                    lineHeight = 9.sp,
                    maxLines = 4,
                    overflow = TextOverflow.Clip
                )
            }
        }

        if (logExpanded) {
            Box(
                modifier = Modifier
                    .fillMaxSize()
                    .zIndex(3f)
                    .pointerInput(Unit) {
                        detectTapGestures { logExpanded = false }
                    }
            )
            Column(
                modifier = Modifier
                    .align(Alignment.TopEnd)
                    .offset(x = (-18).dp, y = 56.dp)
                    .width(430.dp)
                    .height(238.dp)
                    .zIndex(4f)
                    .clip(RoundedCornerShape(7.dp))
                    .background(Color(0xFF111820))
                    .border(1.dp, line, RoundedCornerShape(7.dp))
                    .pointerInput(Unit) {
                        detectTapGestures { }
                    }
                    .padding(horizontal = 10.dp, vertical = 8.dp)
            ) {
                Text(
                    text = "LOG",
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

        if (controlsExpanded) {
            Box(
                modifier = Modifier
                    .fillMaxSize()
                    .zIndex(1f)
                    .pointerInput(Unit) {
                        detectTapGestures { controlsExpanded = false }
                    }
            )
        }

        Box(
            modifier = Modifier
                .align(Alignment.TopStart)
                .width(if (controlsExpanded) 680.dp else 304.dp)
                .height(106.dp)
                .zIndex(2f)
        ) {
            Row(
                modifier = Modifier.width(304.dp),
                horizontalArrangement = Arrangement.spacedBy(6.dp)
            ) {
                Button(
                    onClick = onBack,
                    modifier = Modifier
                        .width(60.dp)
                        .height(42.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = panel2,
                        contentColor = Color.White
                    ),
                    contentPadding = PaddingValues(0.dp),
                    shape = RoundedCornerShape(6.dp)
                ) {
                    Text("Back", fontSize = 11.sp, fontWeight = FontWeight.Bold)
                }

                Button(
                    onClick = onMasterToggle,
                    modifier = Modifier
                        .width(80.dp)
                        .height(42.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (masterEnabled) red else green,
                        contentColor = Color.White
                    ),
                    contentPadding = PaddingValues(0.dp),
                    shape = RoundedCornerShape(6.dp)
                ) {
                    Text(
                        text = if (masterEnabled) "STOP" else "STOP\nONLY",
                        fontWeight = FontWeight.Black,
                        fontSize = 12.sp,
                        lineHeight = 13.sp,
                        maxLines = 2,
                        textAlign = TextAlign.Center
                    )
                }

                DeviceSelector(
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
                    modifier = Modifier.width(138.dp)
                )
            }

            Box(
                modifier = Modifier
                    .offset(y = 66.dp)
                    .width(88.dp)
                    .height(32.dp)
                    .clip(RoundedCornerShape(6.dp))
                    .background(panel2)
                    .border(1.dp, line, RoundedCornerShape(6.dp))
                    .clickable { controlsExpanded = !controlsExpanded }
            ) {
                Row(
                    modifier = Modifier
                    .fillMaxSize()
                    .padding(horizontal = 8.dp),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(
                        text = "Chassis\nConfig",
                        color = Color(0xFFB3C6DE),
                        fontSize = 7.sp,
                        lineHeight = 8.sp,
                        fontWeight = FontWeight.ExtraBold,
                        maxLines = 2
                    )
                    Text(
                        text = if (controlsExpanded) "<" else ">",
                        color = Color.White,
                        fontSize = 16.sp,
                        fontWeight = FontWeight.Black
                    )
                }
            }

            if (controlsExpanded) {
                Row(
                    modifier = Modifier
                        .offset(x = 98.dp, y = 66.dp)
                        .width(386.dp),
                    horizontalArrangement = Arrangement.spacedBy(5.dp)
                ) {
                    ControlPanel("COORDINATE", Modifier.weight(0.78f)) {
                        Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                            SegmentButton(
                                "LCS",
                                mode == CoordinateMode.LCS,
                                blue,
                                Modifier.weight(1f)
                            ) {
                                onModeChange(CoordinateMode.LCS)
                            }
                            SegmentButton(
                                "WCS",
                                mode == CoordinateMode.WCS,
                                blue,
                                Modifier.weight(1f)
                            ) {
                                onModeChange(CoordinateMode.WCS)
                            }
                        }
                    }
                    ControlPanel("ODOM", Modifier.weight(0.52f)) {
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.Center
                        ) {
                            OutlinedButton(
                                onClick = onOdomReset,
                                modifier = Modifier
                                    .width(52.dp)
                                    .height(24.dp),
                                contentPadding = PaddingValues(0.dp),
                                colors = ButtonDefaults.outlinedButtonColors(
                                    contentColor = Color(0xFFFFDCA3)
                                )
                            ) {
                                Text(
                                    "Reset",
                                    fontSize = 8.sp,
                                    lineHeight = 9.sp,
                                    fontWeight = FontWeight.Bold,
                                    textAlign = TextAlign.Center
                                )
                            }
                        }
                    }
                    ScalePanel("SPEED", speedScale, blue, Modifier.weight(0.9f), onSpeedScaleChange)
                    ScalePanel(
                        "ANGULAR",
                        omegaScale,
                        blue,
                        Modifier.weight(1.0f),
                        onOmegaScaleChange
                    )
                }
            }
        }

        JoystickControl(
            speedScale = speedScale,
            enabled = connected && masterEnabled,
            onChange = onJoystickChange,
            modifier = Modifier
                .align(Alignment.BottomStart)
                .offset(x = 46.dp, y = (-38).dp)
        )

        OmegaSlider(
            omegaScale = omegaScale,
            enabled = connected && masterEnabled,
            onChange = onOmegaChange,
            modifier = Modifier
                .align(Alignment.BottomEnd)
                .offset(x = (-40).dp, y = (-42).dp)
        )
    }
}

@Composable
private fun CommandMetric(label: String, value: String, stop: Boolean, modifier: Modifier) {
    Box(
        modifier = modifier
            .height(42.dp)
            .clip(RoundedCornerShape(5.dp))
            .background(Color(0xFF232A34))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(5.dp))
            .padding(horizontal = 6.dp, vertical = 3.dp)
    ) {
        Column(
            modifier = Modifier.fillMaxSize(),
            verticalArrangement = Arrangement.SpaceBetween
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
                color = if (stop) Color(0xFFFFB0AD) else Color.White,
                fontSize = 12.sp,
                lineHeight = 14.sp,
                fontWeight = FontWeight.Black,
                maxLines = 1,
                modifier = Modifier.padding(bottom = 1.dp)
            )
        }
    }
}

@Composable
private fun ControlPanel(title: String, modifier: Modifier, content: @Composable () -> Unit) {
    Column(
        modifier = modifier
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF1B2028))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(6.dp))
            .padding(6.dp)
    ) {
        Text(
            text = title,
            color = Color(0xFFB3C6DE),
            fontSize = 7.sp,
            lineHeight = 8.sp,
            fontWeight = FontWeight.ExtraBold
        )
        Spacer(Modifier.height(4.dp))
        content()
    }
}

@Composable
private fun DeviceSelector(
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

    Column(
        modifier = modifier
            .height(42.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF1B2028))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(6.dp))
            .padding(horizontal = 5.dp, vertical = 4.dp)
    ) {
        Text(
            "BLUETOOTH",
            color = Color(0xFFB3C6DE),
            fontSize = 7.sp,
            lineHeight = 8.sp,
            fontWeight = FontWeight.ExtraBold
        )
        Spacer(Modifier.height(2.dp))
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(4.dp)
        ) {
            Box(Modifier.weight(1f)) {
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(20.dp)
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
                        fontSize = 8.sp,
                        color = Color.White,
                        maxLines = 1,
                        overflow = TextOverflow.Ellipsis,
                        textAlign = TextAlign.Center,
                        modifier = Modifier.fillMaxWidth()
                    )
                }
                DropdownMenu(
                    expanded = menuExpanded,
                    onDismissRequest = { menuExpanded = false }
                ) {
                    if (devices.isEmpty()) {
                        DropdownMenuItem(
                            text = { Text("No bonded devices") },
                            onClick = { menuExpanded = false }
                        )
                    }
                    devices.forEach { device ->
                        DropdownMenuItem(
                            text = {
                                Text(
                                    "${device.name} ${device.address}",
                                    maxLines = 1,
                                    overflow = TextOverflow.Ellipsis
                                )
                            },
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
                    .width(42.dp)
                    .height(20.dp),
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
                    fontSize = 8.sp,
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}

@Composable
private fun ScalePanel(
    title: String,
    selected: Float,
    activeColor: Color,
    modifier: Modifier,
    onChange: (Float) -> Unit
) {
    ControlPanel(title, modifier) {
        Row(horizontalArrangement = Arrangement.spacedBy(3.dp)) {
            listOf(0.25f, 0.5f, 0.75f, 1.0f).forEach { value ->
                SegmentButton(
                    text = if (value == 1.0f) "1x" else (value * 100).toInt().toString(),
                    selected = selected == value,
                    activeColor = activeColor,
                    modifier = Modifier.weight(1f)
                ) {
                    onChange(value)
                }
            }
        }
    }
}

@Composable
private fun SegmentButton(
    text: String,
    selected: Boolean,
    activeColor: Color,
    modifier: Modifier = Modifier,
    onClick: () -> Unit
) {
    Box(
        modifier = modifier
            .height(26.dp)
            .clip(RoundedCornerShape(5.dp))
            .background(if (selected) activeColor else Color(0xFF232A34))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(5.dp))
            .clickable(onClick = onClick),
        contentAlignment = Alignment.Center
    ) {
        Text(text, fontSize = 9.sp, fontWeight = FontWeight.Black, color = Color.White)
    }
}

@Composable
private fun JoystickControl(
    speedScale: Float,
    enabled: Boolean,
    onChange: (Float, Float) -> Unit,
    modifier: Modifier = Modifier
) {
    val density = LocalDensity.current
    var sizePx by remember { mutableIntStateOf(1) }
    var thumbOffset by remember { mutableStateOf(Offset.Zero) }
    val thumbSizePx = with(density) { 48.dp.toPx() }
        val joystickSize = 112.dp

    fun update(position: Offset) {
        if (!enabled) {
            return
        }
        val center = Offset(sizePx / 2f, sizePx / 2f)
        val dx = position.x - center.x
        val dy = center.y - position.y
        val maxRadius = sizePx / 2f - thumbSizePx / 2f
        val distance = hypot(dx, dy).coerceAtMost(maxRadius)
        val screenAngle = atan2(dy, dx)
        val display = Offset(
            x = cos(screenAngle) * distance,
            y = -sin(screenAngle) * distance
        )
        val direction = normalizeDirection((screenAngle - PI / 2.0).toFloat())
        val velocity = (distance / maxRadius) * ControlProtocol.MAX_CHASSIS_V * speedScale

        thumbOffset = display
        onChange(direction, velocity)
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
            .size(joystickSize)
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
            drawCircle(
                Color(0x26FFFFFF),
                radius * 0.62f,
                style = Stroke(width = 1.dp.toPx())
            )
            drawCircle(
                Color(0x22FFFFFF),
                radius * 0.35f,
                style = Stroke(width = 1.dp.toPx())
            )
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
private fun OmegaSlider(
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
        val half = widthPx / 2f - thumbHalfWidth
        val clamped = (x - widthPx / 2f).coerceIn(-half, half)
        val normalized = clamped / half
        thumbX = clamped
        onChange(-normalized * ControlProtocol.MAX_CHASSIS_OMEGA * omegaScale)
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
            .width(180.dp)
            .clip(RoundedCornerShape(7.dp))
            .background(Color(0xFF1B2028))
            .border(1.dp, Color(0xFF36404D), RoundedCornerShape(7.dp))
            .padding(5.dp)
    ) {
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(40.dp)
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
                    .width(52.dp)
                    .height(30.dp)
                    .clip(RoundedCornerShape(50))
                    .background(if (enabled) Color(0xFFE6A23A) else Color(0xFF756246))
                    .border(3.dp, Color.White, RoundedCornerShape(50))
            )
        }
    }
}

private fun normalizeDirection(value: Float): Float {
    var normalized = value % TWO_PI
    if (normalized < 0f) {
        normalized += TWO_PI
    }
    return normalized
}

private fun Float.format3(): String = "%.3f".format(this)

private fun Float.formatDegrees(): String = "%.0f\u00B0".format(this * 180f / PI.toFloat())
