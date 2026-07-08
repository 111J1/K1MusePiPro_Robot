package com.embodiedai.robotcontroller.regions

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class RegionModelsTest {
    private fun freeMap(width: Int = 10, height: Int = 10): OccupancySnapshot {
        return OccupancySnapshot(
            width = width,
            height = height,
            resolution = 0.5f,
            originX = -1.0f,
            originY = -2.0f,
            originYaw = 0f,
            data = ByteArray(width * height) { 0 }
        )
    }

    @Test
    fun mapAndBitmapCoordinatesRoundTrip() {
        val map = freeMap()
        val point = RegionPoint(1.25f, 0.25f)
        val bitmap = map.mapToBitmap(point)
        val roundTrip = map.bitmapToMap(bitmap.x, bitmap.y)

        assertEquals(point.x, roundTrip.x, 0.0001f)
        assertEquals(point.y, roundTrip.y, 0.0001f)
    }

    @Test
    fun centroidAndNearestFreeCenterPreferFreeCell() {
        val map = freeMap()
        val vertices = listOf(
            RegionPoint(0f, 0f),
            RegionPoint(2f, 0f),
            RegionPoint(2f, 2f),
            RegionPoint(0f, 2f)
        )

        val center = navigableCenter(vertices, map)

        assertNotNull(center)
        assertTrue(pointInPolygon(center!!, vertices))
    }

    @Test
    fun nearestFreeCenterReturnsNullWhenRegionHasNoFreeCells() {
        val blocked = freeMap().let {
            it.copy(data = ByteArray(it.width * it.height) { 100.toByte() })
        }
        val vertices = listOf(
            RegionPoint(0f, 0f),
            RegionPoint(1f, 0f),
            RegionPoint(1f, 1f),
            RegionPoint(0f, 1f)
        )

        assertNull(navigableCenter(vertices, blocked))
    }

    @Test
    fun regionDocumentJsonRoundTripSupportsChineseNames() {
        val map = freeMap()
        val region = MapRegion(
            name = "A区",
            color = "#4F8CFF",
            vertices = listOf(
                RegionPoint(0f, 0f),
                RegionPoint(1f, 0f),
                RegionPoint(1f, 1f),
                RegionPoint(0f, 1f)
            )
        )

        val document = buildRegionDocument("manual.yaml", "manual.pgm", map, listOf(region))
        val parsed = parseRegionDocument(document!!.toJsonString())

        assertNotNull(parsed)
        assertEquals("manual.yaml", parsed!!.map.yaml)
        assertEquals("A区", parsed.regions.single().name)
        assertEquals(4, parsed.regions.single().vertices.size)
        assertNotNull(parsed.regions.single().center)
    }

    @Test
    fun blankRegionsJsonMeansMissingSidecar() {
        assertNull(parseRegionDocument(""))
    }
}

