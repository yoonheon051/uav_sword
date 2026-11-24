package com.igeo.wristsword

import android.app.Activity
import android.content.Context
import android.graphics.Color
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.text.InputType
import android.util.TypedValue
import android.view.Gravity
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.Locale

/**
 * 손목 자세(쿼터니언) + 자이로 + 선형가속도를 UDP로 PC에 보내는 앱.
 *
 * 패킷 형식 (CSV 한 줄):
 *   IGEO1,seq,t_ms,qw,qx,qy,qz,gx,gy,gz,lax,lay,laz,drawn
 *   - q   : 워치 좌표계 -> 세계 좌표계 회전 (세계 Z축 = 위쪽)
 *   - g   : 자이로 (rad/s, 워치 좌표계)
 *   - la  : 중력 뺀 선형가속도 (m/s^2, 워치 좌표계)
 *   - drawn: 1 = 발검(검을 띄움), 0 = 회수
 */
class MainActivity : Activity(), SensorEventListener {

    companion object {
        private const val PORT = 5005
        private const val SAMPLE_PERIOD_US = 10_000   // 100 Hz 요청 (실제 속도는 기기가 결정)
        private const val PREFS = "wristsword"
        private const val KEY_HOST = "host"
        private const val DEFAULT_HOST = "255.255.255.255" // 같은 Wi-Fi 전체에 브로드캐스트
    }

    private lateinit var sensorManager: SensorManager
    private lateinit var connectivity: ConnectivityManager

    // 센서 콜백과 네트워크 전송은 모두 이 스레드에서 처리 (메인 스레드에서 네트워크 금지)
    private var worker: HandlerThread? = null
    private var workerHandler: Handler? = null

    private var socket: DatagramSocket? = null
    private var target: InetAddress? = null
    private var wifiCallback: ConnectivityManager.NetworkCallback? = null
    private var onWifi = false

    @Volatile private var streaming = false
    @Volatile private var drawn = false

    private var seq = 0L
    private var sendErrors = 0L
    private var lastUiUpdate = 0L
    private var rateCount = 0
    private var rateStart = 0L
    private var rateHz = 0f

    private val quat = FloatArray(4)
    private val gyro = FloatArray(3)
    private val lin = FloatArray(3)

    private lateinit var statusText: TextView
    private lateinit var hostInput: EditText
    private lateinit var startButton: Button
    private lateinit var drawButton: Button

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // 화면이 꺼지면 센서가 느려지거나 멈추므로 켜둔다.
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        connectivity = getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager

        setContentView(buildUi())
    }

    // ---------------------------------------------------------------- UI

    private fun dp(v: Int): Int =
        TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, v.toFloat(), resources.displayMetrics).toInt()

    private fun buildUi(): ScrollView {
        val prefs = getSharedPreferences(PREFS, MODE_PRIVATE)

        val column = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER_HORIZONTAL
            // 원형 화면 가장자리에 잘리지 않도록 넉넉한 여백
            setPadding(dp(28), dp(24), dp(28), dp(32))
        }

        column.addView(TextView(this).apply {
            text = "以氣御劍"
            textSize = 18f
            setTextColor(Color.WHITE)
            gravity = Gravity.CENTER
        })

        hostInput = EditText(this).apply {
            setText(prefs.getString(KEY_HOST, DEFAULT_HOST))
            textSize = 12f
            hint = "PC IP"
            inputType = InputType.TYPE_CLASS_TEXT
            gravity = Gravity.CENTER
            setSingleLine()
        }
        column.addView(hostInput)

        startButton = Button(this).apply {
            text = "전송 시작"
            setOnClickListener { if (streaming) stopStreaming() else startStreaming() }
        }
        column.addView(startButton)

        drawButton = Button(this).apply {
            text = "발검"
            isEnabled = false
            setOnClickListener {
                drawn = !drawn
                text = if (drawn) "회수" else "발검"
            }
        }
        column.addView(drawButton)

        statusText = TextView(this).apply {
            text = "대기 중"
            textSize = 11f
            setTextColor(Color.LTGRAY)
            gravity = Gravity.CENTER
        }
        column.addView(statusText)

        return ScrollView(this).apply { addView(column) }
    }

    private fun setStatus(s: String) = runOnUiThread { statusText.text = s }

    // ---------------------------------------------------------- streaming

    private fun startStreaming() {
        val host = hostInput.text.toString().trim().ifEmpty { DEFAULT_HOST }
        getSharedPreferences(PREFS, MODE_PRIVATE).edit().putString(KEY_HOST, host).apply()

        streaming = true
        startButton.text = "전송 중지"
        drawButton.isEnabled = true
        hostInput.isEnabled = false

        val thread = HandlerThread("igeo-worker").apply { start() }
        val handler = Handler(thread.looper)
        worker = thread
        workerHandler = handler

        handler.post {
            try {
                target = InetAddress.getByName(host)
                socket = DatagramSocket().apply { broadcast = true }
            } catch (e: Exception) {
                setStatus("주소 오류: ${e.message}")
                runOnUiThread { stopStreaming() }
                return@post
            }
            seq = 0; sendErrors = 0
            rateStart = SystemClock.elapsedRealtime(); rateCount = 0
            requestWifi(handler)
            registerSensors(handler)
            setStatus("전송 중 → $host:$PORT\nWi-Fi 연결 대기…")
        }
    }

    private fun stopStreaming() {
        streaming = false
        sensorManager.unregisterListener(this)
        wifiCallback?.let { runCatching { connectivity.unregisterNetworkCallback(it) } }
        wifiCallback = null

        val thread = worker
        workerHandler?.post {
            socket?.close()
            socket = null
            onWifi = false
            thread?.quitSafely()
        }
        worker = null
        workerHandler = null

        drawn = false
        startButton.text = "전송 시작"
        drawButton.text = "발검"
        drawButton.isEnabled = false
        hostInput.isEnabled = true
        statusText.text = "중지됨"
    }

    /**
     * 갤럭시 워치는 폰과 블루투스로 연결되어 있으면 Wi-Fi를 끄고 폰을 거쳐 통신한다.
     * 그 경로로는 같은 공유기의 PC로 UDP가 안 가므로, Wi-Fi 네트워크를 직접 요청해서
     * 전송 소켓을 거기에 묶는다.
     */
    private fun requestWifi(handler: Handler) {
        val request = NetworkRequest.Builder()
            .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
            // 인터넷이 안 되는 로컬 공유기/핫스팟도 허용
            .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
            .build()

        val cb = object : ConnectivityManager.NetworkCallback() {
            override fun onAvailable(network: Network) {
                // handler 스레드에서 호출됨
                try {
                    val s = DatagramSocket().apply { broadcast = true }
                    network.bindSocket(s)
                    socket?.close()
                    socket = s
                    onWifi = true
                    setStatus("Wi-Fi 연결됨")
                } catch (e: Exception) {
                    setStatus("Wi-Fi 소켓 오류: ${e.message}")
                }
            }

            override fun onLost(network: Network) {
                onWifi = false
                try {
                    socket?.close()
                    socket = DatagramSocket().apply { broadcast = true }
                } catch (_: Exception) { }
                setStatus("Wi-Fi 끊김")
            }
        }
        wifiCallback = cb
        connectivity.requestNetwork(request, cb, handler)
    }

    private fun registerSensors(handler: Handler) {
        val rv = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
            ?: sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
        val gy = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        val la = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)

        if (rv == null) {
            setStatus("회전 센서를 찾을 수 없음")
            return
        }
        // Game Rotation Vector: 지자기를 안 써서 실내 철골/전자기기 근처에서도 덜 튄다.
        // 대신 북쪽 기준이 없으므로 PC에서 '정면 보정(c)'을 해야 한다.
        for (s in listOfNotNull(rv, gy, la)) {
            sensorManager.registerListener(this, s, SAMPLE_PERIOD_US, handler)
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (!streaming) return
        when (event.sensor.type) {
            Sensor.TYPE_GYROSCOPE -> System.arraycopy(event.values, 0, gyro, 0, 3)
            Sensor.TYPE_LINEAR_ACCELERATION -> System.arraycopy(event.values, 0, lin, 0, 3)
            Sensor.TYPE_GAME_ROTATION_VECTOR, Sensor.TYPE_ROTATION_VECTOR -> {
                SensorManager.getQuaternionFromVector(quat, event.values) // [w, x, y, z]
                send(event.timestamp / 1_000_000L)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    private fun send(tMs: Long) {
        val s = socket ?: return
        val t = target ?: return
        val msg = String.format(
            Locale.US,
            "IGEO1,%d,%d,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d",
            seq++, tMs,
            quat[0], quat[1], quat[2], quat[3],
            gyro[0], gyro[1], gyro[2],
            lin[0], lin[1], lin[2],
            if (drawn) 1 else 0
        ).toByteArray()

        try {
            s.send(DatagramPacket(msg, msg.size, t, PORT))
        } catch (e: Exception) {
            sendErrors++
        }

        rateCount++
        val now = SystemClock.elapsedRealtime()
        if (now - rateStart >= 1000) {
            rateHz = rateCount * 1000f / (now - rateStart)
            rateCount = 0
            rateStart = now
        }
        if (now - lastUiUpdate >= 500) {
            lastUiUpdate = now
            val net = if (onWifi) "Wi-Fi" else "Wi-Fi 대기"
            setStatus(String.format(Locale.US, "%s · %.0f Hz\n보냄 %d · 오류 %d", net, rateHz, seq, sendErrors))
        }
    }

    override fun onStop() {
        super.onStop()
        if (streaming) stopStreaming()
    }
}
