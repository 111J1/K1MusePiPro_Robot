package com.embodiedai.robotcontroller.regions

import org.json.JSONArray
import org.json.JSONObject
import java.util.UUID
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

data class RegionPoint(
    val x: Float,
    val y: Float
)

data class MapRegion(
    val id: String = UUID.randomUUID().toString(),
    val name: String,
    val color: String,
    val vertices: List<RegionPoint>,
    val center: RegionPoint? = null,
    val centerType: String = CENTER_TYPE_NEAREST_FREE_CELL
)

data class RegionMapRef(
    val yaml: String,
    val image: String,
    val resolution: Float,
    val originX: Float,
    val originY: Float,
    val originYaw: Float,
    val width: Int,
    val height: Int
)

data class RegionDocument(
    val map: RegionMapRef,
    val regions: List<MapRegion>,
    val schemaVersion: Int = 1
)

data class OccupancySnapshot(
    val width: Int,
    val height: Int,
    val resolution: Float,
    val originX: Float,
    val originY: Float,
    val originYaw: Float,
    val data: ByteArray
) {
    init {
        require(width > 0 && height > 0) { "invalid map size" }
        require(data.size == width * height) { "occupancy data size does not match map size" }
    }

    fun gridToMapCenter(gridX: Int, gridY: Int): RegionPoint {
        return RegionPoint(
            x = originX + (gridX + 0.5f) * resolution,
            y = originY + (gridY + 0.5f) * resolution
        )
    }

    fun mapToGrid(point: RegionPoint): Pair<Int, Int> {
        return Pair(
            floor((point.x - originX) / resolution).toInt(),
            floor((point.y - originY) / resolution).toInt()
        )
    }

    fun mapToBitmap(point: RegionPoint): RegionPoint {
        val grid = mapToGrid(point)
        return RegionPoint(
            x = (point.x - originX) / resolution,
            y = height - 1f - ((point.y - originY) / resolution)
        )
    }

    fun bitmapToMap(bitmapX: Float, bitmapY: Float): RegionPoint {
        return RegionPoint(
            x = originX + bitmapX * resolution,
            y = originY + (height - 1f - bitmapY) * resolution
        )
    }

    fun occupancyAt(gridX: Int, gridY: Int): Int? {
        if (gridX !in 0 until width || gridY !in 0 until height) {
            return null
        }
        return data[gridY * width + gridX].toInt()
    }
}

fun closeable(vertices: List<RegionPoint>): Boolean = vertices.size >= 3

fun polygonCentroid(vertices: List<RegionPoint>): RegionPoint {
    require(vertices.isNotEmpty()) { "vertices must not be empty" }
    if (vertices.size < 3) {
        val sx = vertices.sumOf { it.x.toDouble() }.toFloat()
        val sy = vertices.sumOf { it.y.toDouble() }.toFloat()
        return RegionPoint(sx / vertices.size, sy / vertices.size)
    }

    var twiceArea = 0.0
    var cx = 0.0
    var cy = 0.0
    for (i in vertices.indices) {
        val current = vertices[i]
        val next = vertices[(i + 1) % vertices.size]
        val cross = current.x.toDouble() * next.y.toDouble() - next.x.toDouble() * current.y.toDouble()
        twiceArea += cross
        cx += (current.x + next.x) * cross
        cy += (current.y + next.y) * cross
    }
    if (abs(twiceArea) < 0.000001) {
        val sx = vertices.sumOf { it.x.toDouble() }.toFloat()
        val sy = vertices.sumOf { it.y.toDouble() }.toFloat()
        return RegionPoint(sx / vertices.size, sy / vertices.size)
    }
    return RegionPoint(
        x = (cx / (3.0 * twiceArea)).toFloat(),
        y = (cy / (3.0 * twiceArea)).toFloat()
    )
}

fun pointInPolygon(point: RegionPoint, vertices: List<RegionPoint>): Boolean {
    if (vertices.size < 3) {
        return false
    }
    var inside = false
    var j = vertices.lastIndex
    for (i in vertices.indices) {
        val a = vertices[i]
        val b = vertices[j]
        val intersects = (a.y > point.y) != (b.y > point.y) &&
            point.x < (b.x - a.x) * (point.y - a.y) / ((b.y - a.y).coerceAwayFromZero()) + a.x
        if (intersects) {
            inside = !inside
        }
        j = i
    }
    return inside
}

fun navigableCenter(vertices: List<RegionPoint>, map: OccupancySnapshot): RegionPoint? {
    if (!closeable(vertices)) {
        return null
    }
    val centroid = polygonCentroid(vertices)
    val centroidGrid = map.mapToGrid(centroid)
    if (pointInPolygon(centroid, vertices) && map.occupancyAt(centroidGrid.first, centroidGrid.second) == 0) {
        return map.gridToMapCenter(centroidGrid.first, centroidGrid.second)
    }

    val minX = max(0, floor((vertices.minOf { it.x } - map.originX) / map.resolution).toInt() - 1)
    val maxX = min(map.width - 1, floor((vertices.maxOf { it.x } - map.originX) / map.resolution).toInt() + 1)
    val minY = max(0, floor((vertices.minOf { it.y } - map.originY) / map.resolution).toInt() - 1)
    val maxY = min(map.height - 1, floor((vertices.maxOf { it.y } - map.originY) / map.resolution).toInt() + 1)

    var best: RegionPoint? = null
    var bestDistance = Float.MAX_VALUE
    for (gridY in minY..maxY) {
        for (gridX in minX..maxX) {
            if (map.occupancyAt(gridX, gridY) != 0) {
                continue
            }
            val candidate = map.gridToMapCenter(gridX, gridY)
            if (!pointInPolygon(candidate, vertices)) {
                continue
            }
            val dx = candidate.x - centroid.x
            val dy = candidate.y - centroid.y
            val distance = dx * dx + dy * dy
            if (distance < bestDistance) {
                bestDistance = distance
                best = candidate
            }
        }
    }
    return best
}

fun buildRegionDocument(
    yamlName: String,
    imageName: String,
    map: OccupancySnapshot,
    regions: List<MapRegion>
): RegionDocument? {
    val completed = ArrayList<MapRegion>(regions.size)
    for (region in regions) {
        val center = navigableCenter(region.vertices, map) ?: return null
        completed.add(region.copy(center = center, centerType = CENTER_TYPE_NEAREST_FREE_CELL))
    }
    return RegionDocument(
        map = RegionMapRef(
            yaml = yamlName,
            image = imageName,
            resolution = map.resolution,
            originX = map.originX,
            originY = map.originY,
            originYaw = map.originYaw,
            width = map.width,
            height = map.height
        ),
        regions = completed
    )
}

fun RegionDocument.toJsonString(indent: Int = 2): String {
    val root = JSONObject()
    root.put("schema_version", schemaVersion)
    root.put(
        "map",
        JSONObject()
            .put("yaml", map.yaml)
            .put("image", map.image)
            .put("resolution", map.resolution.toDouble())
            .put("origin", JSONArray().put(map.originX.toDouble()).put(map.originY.toDouble()).put(map.originYaw.toDouble()))
            .put("width", map.width)
            .put("height", map.height)
            .put("image_sha256", "")
    )
    val regionsJson = JSONArray()
    for (region in regions) {
        val verticesJson = JSONArray()
        for (vertex in region.vertices) {
            verticesJson.put(JSONObject().put("x", vertex.x.toDouble()).put("y", vertex.y.toDouble()))
        }
        val center = region.center
        regionsJson.put(
            JSONObject()
                .put("id", region.id)
                .put("name", region.name)
                .put("color", region.color)
                .put("vertices", verticesJson)
                .put("center", JSONObject().put("x", (center?.x ?: 0f).toDouble()).put("y", (center?.y ?: 0f).toDouble()))
                .put("center_type", region.centerType)
        )
    }
    root.put("regions", regionsJson)
    return root.toString(indent)
}

fun parseRegionDocument(json: String): RegionDocument? {
    if (json.isBlank()) {
        return null
    }
    return try {
        val root = JSONObject(json)
        val mapJson = root.getJSONObject("map")
        val origin = mapJson.optJSONArray("origin") ?: JSONArray().put(0.0).put(0.0).put(0.0)
        val regionsJson = root.optJSONArray("regions") ?: JSONArray()
        val regions = ArrayList<MapRegion>(regionsJson.length())
        for (i in 0 until regionsJson.length()) {
            val item = regionsJson.getJSONObject(i)
            val verticesJson = item.getJSONArray("vertices")
            val vertices = ArrayList<RegionPoint>(verticesJson.length())
            for (v in 0 until verticesJson.length()) {
                val vertex = verticesJson.getJSONObject(v)
                vertices.add(RegionPoint(vertex.getDouble("x").toFloat(), vertex.getDouble("y").toFloat()))
            }
            val centerJson = item.optJSONObject("center")
            regions.add(
                MapRegion(
                    id = item.optString("id", UUID.randomUUID().toString()),
                    name = item.optString("name", "区域 ${i + 1}"),
                    color = item.optString("color", DEFAULT_REGION_COLORS[i % DEFAULT_REGION_COLORS.size]),
                    vertices = vertices,
                    center = centerJson?.let {
                        RegionPoint(it.optDouble("x", 0.0).toFloat(), it.optDouble("y", 0.0).toFloat())
                    },
                    centerType = item.optString("center_type", CENTER_TYPE_NEAREST_FREE_CELL)
                )
            )
        }
        RegionDocument(
            schemaVersion = root.optInt("schema_version", 1),
            map = RegionMapRef(
                yaml = mapJson.optString("yaml"),
                image = mapJson.optString("image"),
                resolution = mapJson.optDouble("resolution", 0.0).toFloat(),
                originX = origin.optDouble(0, 0.0).toFloat(),
                originY = origin.optDouble(1, 0.0).toFloat(),
                originYaw = origin.optDouble(2, 0.0).toFloat(),
                width = mapJson.optInt("width", 0),
                height = mapJson.optInt("height", 0)
            ),
            regions = regions
        )
    } catch (_: Exception) {
        null
    }
}

private fun Float.coerceAwayFromZero(): Float {
    return if (abs(this) < 0.000001f) {
        if (this < 0f) -0.000001f else 0.000001f
    } else {
        this
    }
}

const val CENTER_TYPE_NEAREST_FREE_CELL = "nearest_free_cell"
val DEFAULT_REGION_COLORS = listOf("#4F8CFF", "#20A66B", "#E6A23A", "#E53935", "#7D5FFF", "#00A7A7")

