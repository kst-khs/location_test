package com.example.location_test

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Color
import android.graphics.PointF
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.Surface
import android.view.View
import android.view.WindowManager
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.content.PermissionChecker
import androidx.swiperefreshlayout.widget.CircularProgressDrawable
import com.example.location_test.databinding.ActivityFabBinding
import com.example.location_test.databinding.ActivityMainBinding
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.charts.ScatterChart
import com.github.mikephil.charting.data.*
import com.google.android.gms.common.api.GoogleApiClient
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.material.floatingactionbutton.FloatingActionButton
import com.naver.maps.geometry.LatLng
import com.naver.maps.map.*
import com.naver.maps.map.overlay.Marker
import com.naver.maps.map.overlay.OverlayImage
import com.naver.maps.map.overlay.PathOverlay
import kotlin.math.*

class MainActivity : AppCompatActivity(), OnMapReadyCallback {
    private lateinit var sensorManager: SensorManager
    private var accelerometer: Sensor? = null
    private var magnetometer: Sensor? = null

    private var gravity = FloatArray(3)
    private var geomagnetic = FloatArray(3)

    private var rotationMatrix = FloatArray(9)
    private var orientation = FloatArray(3)

    private var azimuth = 0f
    private var correctedAzimuth = 0f
    private var accumulatedAngle = 0f

    private var N = 30
    private val recentHeadings = FloatArray(N)
    private var recentHeadingsIndex = 0
    private var lastHeading = 0f
    private var previousAzimuth = 0f
    private var modeSelect = 0
    private val alpha = 0.1f

    private var trackingEnabled = false
    private var locationEnabled = false
    private var waiting = false
    private var fab: FloatingActionButton? = null
    private var mode: FloatingActionButton? = null
    private var clear: FloatingActionButton? = null
    private var changeGraph: FloatingActionButton? = null
    private lateinit var map: NaverMap

    private lateinit var kalmanFilter: SimpleKalmanFilter
    private lateinit var ekf: ExtendedKalmanFilter
    private lateinit var filter: AlphaBetaGammaFilter

    private lateinit var scatterChart: ScatterChart
    private val entries = ArrayList<Entry>()
    private val entriesScatter = ArrayList<Entry>()
    private val entriesScatter2 = ArrayList<Entry>()
    private var lineChart: LineChart? = null
    private var isLineChartEnabled = true
    private var isScatterChartEnabled = false
    private var time = 0

    private val handler = Handler(Looper.getMainLooper())
    private val updateRunnable = object : Runnable {
        override fun run() {
            updateData()
            drawLineChart()
            drawScatterChart()
            handler.postDelayed(this, 100) // 1초 간격으로 업데이트
        }
    }
    private var latlon = LatLng(35.0, 177.0)
    private val markers = mutableListOf<Marker>()
    private val maxMarkers = 50
    private var handlerMaker: Handler = Handler(Looper.getMainLooper())
    private val updateMarkersRunnable = object : Runnable {
        override fun run() {
            val currentPosition = LatLng(latlon.latitude, latlon.longitude) // 사용자의 현재 위치로 변경하세요.

            addMarker(currentPosition, correctedAzimuth.roundToInt().toFloat())

            // 다음 마커 추가를 예약
            handlerMaker.postDelayed(this, markerUpdateInterval)
        }
    }
    private val markerUpdateInterval: Long = 1000
    private val path = PathOverlay()

    private val locationCallback = object : LocationCallback() {
        override fun onLocationResult(locationResult: LocationResult?) {
            val lastLocation = locationResult?.lastLocation ?: return
            val coord = LatLng(lastLocation)
            val locationOverlay = map.locationOverlay
            locationOverlay.position = coord
            latlon = coord
            locationOverlay.bearing = lastLocation.bearing
            map.moveCamera(CameraUpdate.scrollTo(coord))
            locationOverlay.isVisible = true
        }
    }

    // 칼만필터 값 초기화
    private var r = 15.0f
    private var q = 0.01f
    private var initailP = 10.0f

    // 알파베타감마 값 초기화
    private var alpha1 = 0.1f
    private var beta = 0.0000001f
    private var gamma = 0.0000000000001f

    @SuppressLint("MissingInflatedId")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_fab)

        val mapFragment = supportFragmentManager.findFragmentById(R.id.map_fragment) as MapFragment?
            ?: MapFragment.newInstance().also {
                supportFragmentManager.beginTransaction().add(R.id.map_fragment, it).commit()
            }
        mapFragment.getMapAsync(this)

        // 라인차트
        lineChart= findViewById(R.id.lineChart)
        handler.post(updateRunnable)

        handlerMaker.post(updateMarkersRunnable)
        // 스캐터 차트
        scatterChart = findViewById(R.id.scatterChart)
        scatterChart.visibility = if (isScatterChartEnabled) View.VISIBLE else View.GONE

        // 평균 필터 기본 30개 -> 300개
        N = 30

        // 칼만 필터 초기화
        kalmanFilter = SimpleKalmanFilter(r, q, initailP)
        ekf = ExtendedKalmanFilter(
            r = r,
            q = q,
            initialP = initailP,
            f = { x -> x }, // 선형 상태 전이 함수: x(k) = x(k-1)
            h = { x -> x }, // 선형 측정 함수: z(k) = x(k)
            df = { x -> 1.0F }, // 상태 전이 함수의 미분: dx/dx = 1
            dh = { x -> 1.0F } // 측정 함수의 미분: dz/dx = 1
        )

        // 알파베타감마 필터 초기화
        filter = AlphaBetaGammaFilter(alpha1, beta, gamma)

        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)

        fab = findViewById(R.id.fab)
        fab?.setImageResource(R.drawable.ic_my_location_black_24dp)
        mode = findViewById(R.id.mode)
        clear = findViewById(R.id.clearGraph)
        changeGraph = findViewById(R.id.changeGraph)
    }
    private fun updateData() {
        entries.add(Entry((entries.size + 1).toFloat(), correctedAzimuth.roundToInt().toFloat()))
        entriesScatter.add(Entry((++time).toFloat(), correctedAzimuth.roundToInt().toFloat()))
        entriesScatter2.add(Entry((time).toFloat(), azimuth.roundToInt().toFloat()))
    }
    private fun drawLineChart() {
        val dataSet = LineDataSet(entries, "Real-time Data")
        val lineData = LineData(dataSet)
        lineChart?.data = lineData
        lineChart?.invalidate()
    }
    private fun drawScatterChart() {
        val scatterDataSet = ScatterDataSet(entriesScatter, "필터")
        scatterDataSet.color = Color.RED
        scatterDataSet.setScatterShape(ScatterChart.ScatterShape.CIRCLE)

        // 두 번째 데이터 세트
        val scatterDataSet2 = ScatterDataSet(entriesScatter2, "원본")
        scatterDataSet2.color = Color.BLUE
        scatterDataSet2.setScatterShape(ScatterChart.ScatterShape.SQUARE)

        val scatterData = ScatterData(scatterDataSet, scatterDataSet2)
        scatterChart.data = scatterData
        scatterChart.invalidate()
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        grantResults: IntArray
    ) {
        if (requestCode == PERMISSION_REQUEST_CODE) {
            if (grantResults.all { it == PermissionChecker.PERMISSION_GRANTED }) {
                enableLocation()
            } else {
                fab?.setImageResource(R.drawable.ic_my_location_black_24dp)
            }
            return
        }

        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
    }
    override fun onResume() {
        super.onResume()
        accelerometer?.also { sensor ->
            sensorManager.registerListener(
                sensorListener, sensor, SensorManager.SENSOR_DELAY_NORMAL
            )
        }
        magnetometer?.also { sensor ->
            sensorManager.registerListener(
                sensorListener, sensor, SensorManager.SENSOR_DELAY_NORMAL
            )
        }
    }
    override fun onStart() {
        super.onStart()
        if (trackingEnabled) {
            enableLocation()
        }
    }
    override fun onStop() {
        super.onStop()
        disableLocation()
        handlerMaker.removeCallbacks(updateMarkersRunnable)
    }
    override fun onMapReady(naverMap: NaverMap) {
        map = naverMap
        val locationOverlay = map.locationOverlay
        locationOverlay.icon = OverlayImage.fromResource(R.drawable.arrow)
        locationOverlay.iconHeight = 150
        locationOverlay.iconWidth = 150

        fab?.setOnClickListener {
            if (trackingEnabled) {
                disableLocation()
                fab?.setImageResource(R.drawable.ic_my_location_black_24dp)
            } else {
                fab?.setImageDrawable(CircularProgressDrawable(this).apply {
                    setStyle(CircularProgressDrawable.LARGE)
                    setColorSchemeColors(Color.WHITE)
                    start()
                })
                tryEnableLocation()
            }
            trackingEnabled = !trackingEnabled
        }

        mode?.setOnClickListener {
            modeSelect++
        }

        clear?.setOnClickListener {
            time = 0
            entries.clear()
            entriesScatter.clear()
            entriesScatter2.clear()
        }

        changeGraph?.setOnClickListener {
            isLineChartEnabled = !isLineChartEnabled
            isScatterChartEnabled = !isScatterChartEnabled
            lineChart?.visibility = if (isLineChartEnabled) View.VISIBLE else View.GONE
            scatterChart.visibility = if (isScatterChartEnabled) View.VISIBLE else View.GONE
        }
    }
    private fun addMarker(position: LatLng, angle: Float) {
        // 마커 생성 및 설정
        val marker = Marker().apply {
            this.position = position
            icon = OverlayImage.fromResource(R.drawable.arrow)
            anchor = PointF(0.5f, 0.5f)
            this.angle = angle
            alpha = 0.5f
            width = 50
            height = 50
            isFlat = true
        }

        // 지도에 마커 추가
        if (::map.isInitialized) {
            marker.map = map
        }

        // 마커 목록에 추가
        markers.add(marker)

        // 마커가 maxMarkers를 초과하면 가장 오래된 마커를 지도에서 제거하고 목록에서 삭제
        if (markers.size > maxMarkers) {
            markers[0].map = null
            markers.removeAt(0)
        }
        // 지도에 path 추가
        val markerLoca = mutableListOf<LatLng>()
        markers.forEach {e ->
            markerLoca.add(e.position)
        }
        //path.coords = markerLoca
        //path.map = map
    }

    private fun tryEnableLocation() {
        if (PERMISSIONS.all {
                ContextCompat.checkSelfPermission(
                    this,
                    it
                ) == PermissionChecker.PERMISSION_GRANTED
            }) {
            enableLocation()
        } else {
            ActivityCompat.requestPermissions(this, PERMISSIONS, PERMISSION_REQUEST_CODE)
        }
    }
    private fun enableLocation() {
        GoogleApiClient.Builder(this)
            .addConnectionCallbacks(object : GoogleApiClient.ConnectionCallbacks {
                @SuppressLint("MissingPermission", "VisibleForTests")
                override fun onConnected(bundle: Bundle?) {
                    val locationRequest = LocationRequest().apply {
                        priority = LocationRequest.PRIORITY_HIGH_ACCURACY
                        interval = LOCATION_REQUEST_INTERVAL.toLong()
                        fastestInterval = LOCATION_REQUEST_INTERVAL.toLong()
                    }

                    LocationServices.getFusedLocationProviderClient(this@MainActivity)
                        .requestLocationUpdates(locationRequest, locationCallback, null)
                    locationEnabled = true
                    waiting = true
                }

                override fun onConnectionSuspended(i: Int) {
                }
            })
            .addApi(LocationServices.API)
            .build()
            .connect()
    }
    private fun disableLocation() {
        if (!locationEnabled) {
            return
        }
        LocationServices.getFusedLocationProviderClient(this)
            .removeLocationUpdates(locationCallback)
        locationEnabled = false
    }
    private val sensorListener = object : SensorEventListener {
        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        }

        @SuppressLint("SetTextI18n")
        override fun onSensorChanged(event: SensorEvent?) {
            when (event?.sensor?.type) {
                Sensor.TYPE_ACCELEROMETER -> gravity = event.values.clone()
                Sensor.TYPE_MAGNETIC_FIELD -> geomagnetic = event.values.clone()
            }

            val success = SensorManager.getRotationMatrix(
                rotationMatrix, null, gravity, geomagnetic
            )

            if (success) {
                SensorManager.getOrientation(rotationMatrix, orientation)
                azimuth = Math.toDegrees(orientation[0].toDouble()).toFloat()
                updateCorrectedAzimuth()

                // 평균 필터
                if(modeSelect % 3 == 0) {
                    if (::map.isInitialized) {
                        updateHeadingAverage(correctedAzimuth)
                    }
                }

                // 알파-베타-감마 필터
                if (modeSelect % 3 == 1) {
                    val dt = 0.01f // 샘플링 간격(시간) - 실제 시간 간격으로 변경해야 함
                    correctedAzimuth = filter.update(correctedAzimuth, dt)
                    map.locationOverlay.bearing = correctedAzimuth
                    val cameraUpdate = CameraUpdate.withParams(
                        CameraUpdateParams()
                            .scrollTo(
                                LatLng(
                                    map.locationOverlay.position.latitude,
                                    map.locationOverlay.position.longitude
                                )
                            )
                            .rotateTo(correctedAzimuth.roundToInt().toDouble())
                    ).animate(CameraAnimation.None)
                    map.moveCamera(cameraUpdate)
                }


                // 칼만 필터
                if (modeSelect % 3 == 2) {
                    updateHeadingKalman(correctedAzimuth)  // 칼만 필터로 heading 값 보정
                }

                val roundedGravity = gravity.map { round(it * 100) / 100 }.toFloatArray()
                val tvText1 = findViewById<TextView>(R.id.tvText1)
                val tvText2 = findViewById<TextView>(R.id.tvText2)
                val tvText3 = findViewById<TextView>(R.id.tvText3)

                tvText1.text = "%.2f,   %.2f,   %.2f".format(roundedGravity[0], roundedGravity[1], roundedGravity[2])
                tvText2.text = "${correctedAzimuth.roundToInt() % 360}, $previousAzimuth / " +
                        when(previousAzimuth) {
                            1.0f -> "+90"
                            2.0f -> "-90"
                            3.0f -> "+180"
                            else -> "0"
                        }
                when(modeSelect % 3) {
                    0 -> tvText3.text = "평균 필터"
                    1 -> tvText3.text = "알파 베타 감마 필터"
                    2 -> tvText3.text = "칼만 필터"
                    else -> tvText3.text = "Error"
                }
            }
        }
    }
    @SuppressLint("SetTextI18n")
    private fun updateCorrectedAzimuth() {
        val windowManager = getSystemService(Context.WINDOW_SERVICE) as WindowManager
        val display = windowManager.defaultDisplay

        when (display.rotation) {
            Surface.ROTATION_0 -> correctedAzimuth = azimuth
            Surface.ROTATION_90 -> correctedAzimuth = azimuth + 90
            Surface.ROTATION_180 -> correctedAzimuth = azimuth + 180
            Surface.ROTATION_270 -> correctedAzimuth = azimuth + 270
        }

        if (gravity[1] >= 9.79f) {
            gravity[1] = 9.79f
        }

        if (gravity[1] >= 9.79) {
            correctedAzimuth += 90
            previousAzimuth = 1.0f
        } else if (gravity[1] <= -9.79) {
            correctedAzimuth -= 90

            previousAzimuth = 2.0f
        } else if (gravity[2] <= 0) {
            correctedAzimuth += 180
            previousAzimuth = 3.0f
        } else {
            previousAzimuth = 0.0f
        }
    }
    @SuppressLint("SetTextI18n")
    private fun updateHeadingAverage(newHeading: Float) {
        if (gravity[2] >= -0.1 && gravity[2] <= 0.1) return
        if (lastHeading != 0f) {
            val angleDiff = getMinimalAngleDifference(newHeading, lastHeading)
            recentHeadings[recentHeadingsIndex] = (lastHeading + angleDiff) % 360
        } else {
            recentHeadings[recentHeadingsIndex] = newHeading
        }
        recentHeadingsIndex = (recentHeadingsIndex + 1) % N

        lastHeading = recentHeadings.average().toFloat()
        correctedAzimuth = recentHeadings.average().toFloat()

        map.locationOverlay.bearing = correctedAzimuth
        val cameraUpdate = CameraUpdate.withParams(
            CameraUpdateParams()
                .scrollTo(
                    LatLng(
                        map.locationOverlay.position.latitude,
                        map.locationOverlay.position.longitude
                    )
                )
                .rotateTo(correctedAzimuth.roundToInt().toDouble())
        ).animate(CameraAnimation.None)
        map.moveCamera(cameraUpdate)
        Log.i("headingTest", "updateHeadingAverage: $correctedAzimuth")
    }
    private fun lowPassFilter(input: Float, output: Float): Float {
        return output + alpha * (input - output)
    }
    private fun updateHeadingKalman(newHeading: Float) {
       if (gravity[2] >= -0.1 && gravity[2] <= 0.1) return
       val angleDiff = getMinimalAngleDifference(newHeading, lastHeading)
       accumulatedAngle += angleDiff
       correctedAzimuth = (lastHeading + angleDiff)

        correctedAzimuth = lowPassFilter(correctedAzimuth, lastHeading)

        // 칼만 필터 업데이트
        //correctedAzimuth = kalmanFilter.update(correctedAzimuth)
        correctedAzimuth = ekf.update(correctedAzimuth)
        lastHeading = correctedAzimuth

        // 보정된 heading 값 출력
        map.locationOverlay.bearing = correctedAzimuth.roundToInt().toFloat()
        val cameraUpdate = CameraUpdate.withParams(
            CameraUpdateParams()
                .scrollTo(
                    LatLng(
                        map.locationOverlay.position.latitude,
                        map.locationOverlay.position.longitude
                    )
                )
                .rotateTo(correctedAzimuth.roundToInt().toDouble())
        ).animate(CameraAnimation.None)
        map.moveCamera(cameraUpdate)
        Log.i("headingTest", "updateHeadingKalman: ${correctedAzimuth.roundToInt() % 360}")
    }
    private fun getMinimalAngleDifference(angle1: Float, angle2: Float): Float {
        val diff = (angle1 - angle2 + 180) % 360 -180
        return if (diff < -180) diff + 360 else diff
    }
    companion object {
        private const val LOCATION_REQUEST_INTERVAL = 100
        private const val PERMISSION_REQUEST_CODE = 100
        private val PERMISSIONS = arrayOf(
            android.Manifest.permission.ACCESS_FINE_LOCATION,
            android.Manifest.permission.ACCESS_COARSE_LOCATION,
        )
    }
}
