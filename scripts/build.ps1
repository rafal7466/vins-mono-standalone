<#
.SYNOPSIS
  Configures and builds vins-mono-standalone (MSVC + vcpkg, CMake preset "vs2022-vcpkg").

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File scripts\build.ps1
#>
param(
  [ValidateSet("Release", "Debug")]
  [string]$Config = "Release"
)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
Push-Location $repo
try {
  $start = Get-Date
  cmake --preset vs2022-vcpkg
  if ($LASTEXITCODE -ne 0) { throw "CMake configure failed (is vcpkg installed? see scripts\setup_vcpkg.ps1)" }
  cmake --build build --config $Config --parallel
  if ($LASTEXITCODE -ne 0) { throw "Build failed" }
  $exe = Join-Path $repo "build\$Config\vins_euroc.exe"
  if (-not (Test-Path $exe)) { throw "Build finished but $exe was not found" }
  Write-Host ("Built {0} in {1:N0} s" -f $exe, ((Get-Date) - $start).TotalSeconds)
} finally {
  Pop-Location
}
