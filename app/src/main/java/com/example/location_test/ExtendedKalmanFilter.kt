package com.example.location_test

import kotlin.math.sin
import kotlin.math.cos

class ExtendedKalmanFilter(
    private val r: Float,
    private val q: Float,
    private val initialP: Float,
    private val f: (Float) -> Float = { it }, // 상태 전이 함수, 기본적으로 선형입니다.
    private val h: (Float) -> Float = { it }, // 측정 전이 함수, 기본적으로 선형입니다.
    private val df: (Float) -> Float = { 1f }, // 상태 전이 함수의 미분
    private val dh: (Float) -> Float = { 1f } // 측정 전이 함수의 미분
) {
    private var x = 0f
    private var p = initialP

    fun update(measurement: Float): Float {
        // 예측 단계
        val xPredicted = f(x)
        val pPredicted = df(x) * p * df(x) + q

        // 업데이트 단계
        val k = pPredicted * dh(xPredicted) / (dh(xPredicted) * pPredicted * dh(xPredicted) + r)
        x = xPredicted + k * (measurement - h(xPredicted))
        p = (1 - k * dh(xPredicted)) * pPredicted

        return x
    }
}
