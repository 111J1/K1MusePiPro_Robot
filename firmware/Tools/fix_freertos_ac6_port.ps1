param(
    [string]$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$Uv2CsolutionPath,
    [switch]$SkipUv2Csolution,
    [switch]$KeepCbuildCache
)

$ErrorActionPreference = "Stop"

$backupPortDir = Join-Path $ProjectRoot "Tools\FreeRTOS_AC6_Port\ARM_CM4F"
$backupPortC = Join-Path $backupPortDir "port.c"
$backupPortMacro = Join-Path $backupPortDir "portmacro.h"

$gccPortDir = Join-Path $ProjectRoot "Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM4F"
$portC = Join-Path $gccPortDir "port.c"
$portMacro = Join-Path $gccPortDir "portmacro.h"
$freertosIncludeDir = Join-Path $ProjectRoot "Middlewares\Third_Party\FreeRTOS\Source\include"
$includePortMacro = Join-Path $freertosIncludeDir "portmacro.h"

if (!(Test-Path -LiteralPath $backupPortC) -or !(Test-Path -LiteralPath $backupPortMacro)) {
    throw "Missing backup GCC ARM_CM4F FreeRTOS port files in: $backupPortDir"
}

New-Item -ItemType Directory -Force -Path $gccPortDir | Out-Null
Copy-Item -LiteralPath $backupPortC -Destination $portC -Force
Copy-Item -LiteralPath $backupPortMacro -Destination $portMacro -Force
Write-Host "Restored GCC ARM_CM4F FreeRTOS port files to: $gccPortDir"

if (Test-Path -LiteralPath $freertosIncludeDir) {
    Copy-Item -LiteralPath $backupPortMacro -Destination $includePortMacro -Force
    Write-Host "Installed GCC portmacro.h fallback to: $includePortMacro"
} else {
    Write-Warning "FreeRTOS include directory not found: $freertosIncludeDir"
}

$projectFiles = @(
    (Join-Path $ProjectRoot "MDK-ARM\EmbodiedAI_Robot.uvprojx"),
    (Join-Path $ProjectRoot "MDK-ARM\EmbodiedAI_Robot.uvoptx")
)

$uvprojx = Join-Path $ProjectRoot "MDK-ARM\EmbodiedAI_Robot.uvprojx"
$cproject = Join-Path $ProjectRoot "MDK-ARM\EmbodiedAI_Robot.cproject.yml"
$csolution = Join-Path $ProjectRoot "MDK-ARM\EmbodiedAI_Robot.csolution.yml"

foreach ($file in $projectFiles) {
    if (!(Test-Path -LiteralPath $file)) {
        Write-Warning "Skip missing file: $file"
        continue
    }

    $text = Get-Content -LiteralPath $file -Raw
    $updated = $text `
        -replace "portable/RVDS/ARM_CM4F", "portable/GCC/ARM_CM4F" `
        -replace "portable\\RVDS\\ARM_CM4F", "portable\\GCC\\ARM_CM4F"

    if ($updated -ne $text) {
        Set-Content -LiteralPath $file -Value $updated -NoNewline -Encoding UTF8
        Write-Host "Updated: $file"
    } else {
        Write-Host "Already OK: $file"
    }
}

$missing = @()
foreach ($required in @($portC, $portMacro, $includePortMacro)) {
    if (!(Test-Path -LiteralPath $required)) {
        $missing += $required
    }
}

if ($missing.Count -gt 0) {
    throw "FreeRTOS AC6 port patch incomplete. Missing: $($missing -join ', ')"
}

if (!$SkipUv2Csolution) {
    $uv2 = $null

    if (![string]::IsNullOrWhiteSpace($Uv2CsolutionPath)) {
        $candidate = $Uv2CsolutionPath
        if (![System.IO.Path]::IsPathRooted($candidate)) {
            $candidate = Join-Path $ProjectRoot $candidate
        }

        if (Test-Path -LiteralPath $candidate) {
            $uv2 = (Resolve-Path -LiteralPath $candidate).Path
        } else {
            throw "Specified uv2csolution.exe not found: $candidate"
        }
    }

    if ($null -eq $uv2) {
        $cmd = Get-Command uv2csolution.exe -ErrorAction SilentlyContinue
        if ($cmd) {
            $uv2 = $cmd.Source
        }
    }

    if ($null -eq $uv2) {
        $relativeUv2Candidates = @(
            "MDK-ARM\uv2csolution.exe",
            "Tools\uv2csolution.exe",
            "..\UV4\uv2csolution.exe",
            "..\..\UV4\uv2csolution.exe",
            "..\..\Keil5\UV4\uv2csolution.exe",
            "..\..\..\Keil5\UV4\uv2csolution.exe",
            ".vcpkg\artifacts\tools.arm.mdk.toolbox\bin\uv2csolution.exe"
        )

        foreach ($relativeCandidate in $relativeUv2Candidates) {
            $candidate = Join-Path $ProjectRoot $relativeCandidate
            if (Test-Path -LiteralPath $candidate) {
                $uv2 = (Resolve-Path -LiteralPath $candidate).Path
                break
            }
        }
    }

    if ($null -eq $uv2) {
        $ancestor = Get-Item -LiteralPath $ProjectRoot
        while ($null -ne $ancestor) {
            $ancestorCandidates = @(
                (Join-Path $ancestor.FullName "Keil5\UV4\uv2csolution.exe"),
                (Join-Path $ancestor.FullName "UV4\uv2csolution.exe")
            )

            foreach ($candidate in $ancestorCandidates) {
                if (Test-Path -LiteralPath $candidate) {
                    $uv2 = (Resolve-Path -LiteralPath $candidate).Path
                    break
                }
            }

            if ($null -ne $uv2) {
                break
            }

            $ancestor = $ancestor.Parent
        }
    }

    if ($null -eq $uv2) {
        throw "uv2csolution.exe not found. Add it to PATH, pass -Uv2CsolutionPath <relative-or-absolute-path>, or pass -SkipUv2Csolution."
    }

    Write-Host "Using uv2csolution: $uv2"
    Write-Host "Regenerating CMSIS csolution/cproject from: $uvprojx"
    Push-Location $ProjectRoot
    try {
        & $uv2 $uvprojx
        if ($LASTEXITCODE -ne 0) {
            throw "uv2csolution failed with exit code $LASTEXITCODE"
        }
    } finally {
        Pop-Location
    }

    if (!(Test-Path -LiteralPath $cproject) -or !(Test-Path -LiteralPath $csolution)) {
        throw "uv2csolution did not generate expected csolution/cproject files."
    }
}

if (!$KeepCbuildCache) {
    $cacheDirs = @(
        (Join-Path $ProjectRoot "MDK-ARM\tmp"),
        (Join-Path $ProjectRoot "MDK-ARM\out")
    )

    $mdkRoot = (Resolve-Path (Join-Path $ProjectRoot "MDK-ARM")).Path
    foreach ($dir in $cacheDirs) {
        if (Test-Path -LiteralPath $dir) {
            $resolved = (Resolve-Path -LiteralPath $dir).Path
            if (!$resolved.StartsWith($mdkRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
                throw "Refusing to remove cache outside MDK-ARM: $resolved"
            }
            try {
                Remove-Item -LiteralPath $resolved -Recurse -Force -ErrorAction Stop
                Write-Host "Removed cbuild cache: $resolved"
            } catch {
                Write-Warning "Could not remove cbuild cache '$resolved'. Close VS Code/Arm Pack build tasks if stale build output persists. $($_.Exception.Message)"
            }
        }
    }
}

if (Test-Path -LiteralPath $cproject) {
    $cprojectText = Get-Content -LiteralPath $cproject -Raw
    $requiredCprojectText = @(
        "../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
        "../Middlewares/Third_Party/FreeRTOS/Source/include",
        "../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
        "../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"
    )

    foreach ($needle in $requiredCprojectText) {
        if (!$cprojectText.Contains($needle)) {
            throw "cproject.yml is missing required FreeRTOS entry: $needle"
        }
    }
}

Write-Host "FreeRTOS AC6 GCC/ARM_CM4F port patch applied."
