package com.embodiedai.robotcontroller

import androidx.compose.ui.geometry.Offset
import org.junit.Assert.assertEquals
import org.junit.Test

class K1MapViewportTest {
    @Test
    fun rotationKeepsMapScaleStable() {
        val zero = computeMapViewport(
            canvasWidth = 800f,
            canvasHeight = 480f,
            bitmapWidth = 320f,
            bitmapHeight = 180f,
            rotationTurns = 0,
            scale = 1f,
            pan = Offset.Zero
        )
        val quarter = computeMapViewport(
            canvasWidth = 800f,
            canvasHeight = 480f,
            bitmapWidth = 320f,
            bitmapHeight = 180f,
            rotationTurns = 1,
            scale = 1f,
            pan = Offset.Zero
        )

        assertEquals(zero.drawScale, quarter.drawScale, 0.0001f)
    }

    @Test
    fun rotationKeepsMapCentered() {
        val viewport = computeMapViewport(
            canvasWidth = 800f,
            canvasHeight = 480f,
            bitmapWidth = 320f,
            bitmapHeight = 180f,
            rotationTurns = 1,
            scale = 1f,
            pan = Offset.Zero
        )
        val centerX = viewport.topLeft.x + viewport.rotatedWidth * viewport.drawScale / 2f
        val centerY = viewport.topLeft.y + viewport.rotatedHeight * viewport.drawScale / 2f

        assertEquals(400f, centerX, 0.0001f)
        assertEquals(240f, centerY, 0.0001f)
    }

    @Test
    fun allRotationsFitInsideCanvas() {
        for (turn in 0..3) {
            val viewport = computeMapViewport(
                canvasWidth = 800f,
                canvasHeight = 480f,
                bitmapWidth = 320f,
                bitmapHeight = 180f,
                rotationTurns = turn,
                scale = 1f,
                pan = Offset.Zero
            )

            assert(viewport.rotatedWidth * viewport.drawScale <= 800f + 0.0001f)
            assert(viewport.rotatedHeight * viewport.drawScale <= 480f + 0.0001f)
        }
    }

    @Test
    fun panOffsetsCenteredMapExplicitly() {
        val viewport = computeMapViewport(
            canvasWidth = 800f,
            canvasHeight = 480f,
            bitmapWidth = 320f,
            bitmapHeight = 180f,
            rotationTurns = 2,
            scale = 1.5f,
            pan = Offset(24f, -16f)
        )
        val centerX = viewport.topLeft.x + viewport.rotatedWidth * viewport.drawScale / 2f
        val centerY = viewport.topLeft.y + viewport.rotatedHeight * viewport.drawScale / 2f

        assertEquals(424f, centerX, 0.0001f)
        assertEquals(224f, centerY, 0.0001f)
    }

    @Test
    fun labelAnchorFollowsMapRotation() {
        val viewport = computeMapViewport(
            canvasWidth = 800f,
            canvasHeight = 480f,
            bitmapWidth = 100f,
            bitmapHeight = 60f,
            rotationTurns = 1,
            scale = 1f,
            pan = Offset.Zero
        )

        val point = mapPointToCanvas(
            bitmapPoint = Offset(20f, 10f),
            bitmapWidth = 100f,
            bitmapHeight = 60f,
            rotationTurns = 1,
            viewport = viewport
        )

        assertEquals(viewport.topLeft.x + 50f * viewport.drawScale, point.x, 0.0001f)
        assertEquals(viewport.topLeft.y + 20f * viewport.drawScale, point.y, 0.0001f)
    }
}
