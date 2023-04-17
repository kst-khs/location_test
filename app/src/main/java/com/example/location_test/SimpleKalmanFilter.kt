package com.example.location_test

class SimpleKalmanFilter(private val r: Float, private val q: Float, private val initialP: Float) {
    private var x = 0f
    private var p = initialP

    fun update(measurement: Float): Float {
        val k = p / (p + r)
        x += k * (measurement - x)
        p = (1 - k) * p + q

        return x
    }
}
