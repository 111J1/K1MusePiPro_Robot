package com.embodiedai.robotcontroller

import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.performClick
import com.embodiedai.robotcontroller.protocol.K1MapLibraryEntry
import org.junit.Rule
import org.junit.Test

class K1RegionEditOverlayTest {
    @get:Rule
    val compose = createComposeRule()

    @Test
    fun controlsAreDisabledWithoutBridgeOrMap() {
        compose.setContent {
            K1RegionEditOverlay(
                libraryMaps = emptyList(),
                selectedLibraryMap = null,
                regions = emptyList(),
                selectedRegionId = null,
                draftPointCount = 0,
                onlineBridge = false,
                libraryBusy = false,
                regionStatus = "No map selected",
                onRefreshLibraryMaps = {},
                onSelectLibraryMap = {},
                onLoadLibraryMap = {},
                onAddRegion = {},
                onUndoRegionPoint = {},
                onCloseRegion = {},
                onRenameRegion = {},
                onDeleteRegion = {},
                onRedrawRegion = {},
                onSaveRegions = {},
                onCancelEditChanges = {},
                onSelectRegion = {}
            )
        }

        compose.onNodeWithTag("regions-refresh-maps").assertIsNotEnabled()
        compose.onNodeWithTag("regions-load-map").assertIsNotEnabled()
        compose.onNodeWithTag("regions-add").assertIsNotEnabled()
        compose.onNodeWithTag("regions-save").assertIsNotEnabled()
    }

    @Test
    fun coreControlsEnableWithBridgeAndSelectedMap() {
        val map = K1MapLibraryEntry("manual.yaml", "manual.pgm", false)
        compose.setContent {
            K1RegionEditOverlay(
                libraryMaps = listOf(map),
                selectedLibraryMap = map,
                regions = emptyList(),
                selectedRegionId = null,
                draftPointCount = 3,
                onlineBridge = true,
                libraryBusy = false,
                regionStatus = "ready",
                onRefreshLibraryMaps = {},
                onSelectLibraryMap = {},
                onLoadLibraryMap = {},
                onAddRegion = {},
                onUndoRegionPoint = {},
                onCloseRegion = {},
                onRenameRegion = {},
                onDeleteRegion = {},
                onRedrawRegion = {},
                onSaveRegions = {},
                onCancelEditChanges = {},
                onSelectRegion = {}
            )
        }

        compose.onNodeWithTag("regions-refresh-maps").assertIsEnabled()
        compose.onNodeWithTag("regions-load-map").assertIsEnabled()
        compose.onNodeWithTag("regions-add").assertIsEnabled()
        compose.onNodeWithTag("regions-close").assertIsEnabled()
        compose.onNodeWithTag("regions-save").assertIsEnabled()
        compose.onNodeWithTag("regions-rename").assertIsNotEnabled()
    }

    @Test
    fun regionSelectionEnablesEditActions() {
        val map = K1MapLibraryEntry("manual.yaml", "manual.pgm", true)
        val region = com.embodiedai.robotcontroller.regions.MapRegion(
            id = "r1",
            name = "A区",
            color = "#4F8CFF",
            vertices = listOf(
                com.embodiedai.robotcontroller.regions.RegionPoint(0f, 0f),
                com.embodiedai.robotcontroller.regions.RegionPoint(1f, 0f),
                com.embodiedai.robotcontroller.regions.RegionPoint(1f, 1f)
            )
        )
        var selected = ""
        compose.setContent {
            K1RegionEditOverlay(
                libraryMaps = listOf(map),
                selectedLibraryMap = map,
                regions = listOf(region),
                selectedRegionId = "r1",
                draftPointCount = 0,
                onlineBridge = true,
                libraryBusy = false,
                regionStatus = "ready",
                onRefreshLibraryMaps = {},
                onSelectLibraryMap = {},
                onLoadLibraryMap = {},
                onAddRegion = {},
                onUndoRegionPoint = {},
                onCloseRegion = {},
                onRenameRegion = {},
                onDeleteRegion = {},
                onRedrawRegion = {},
                onSaveRegions = {},
                onCancelEditChanges = {},
                onSelectRegion = { selected = it }
            )
        }

        compose.onNodeWithTag("region-row-A区").performClick()
        compose.onNodeWithTag("regions-rename").assertIsEnabled()
        compose.onNodeWithTag("regions-delete").assertIsEnabled()
        compose.onNodeWithTag("regions-redraw").assertIsEnabled()
        assert(selected == "r1")
    }
}
