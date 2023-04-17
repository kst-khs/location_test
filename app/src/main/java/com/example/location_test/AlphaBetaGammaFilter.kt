package com.example.location_test

class AlphaBetaGammaFilter(private val alpha: Float, private val beta: Float, private val gamma: Float) {
    private var value: Float = 0f
    private var derivative: Float = 0f
    private var acceleration: Float = 0f

    init {
        reset()
    }

    fun reset() {
        value = 0f
        derivative = 0f
        acceleration = 0f
    }

    fun update(measurement: Float, dt: Float): Float {
        val newValue = value + derivative * dt + 0.5f * acceleration * dt * dt
        val newDerivative = derivative + acceleration * dt
        val residual = measurement - newValue

        value = newValue + alpha * residual
        derivative = newDerivative + beta * residual / dt
        acceleration += gamma * residual / (dt * dt)

        return value
    }
}