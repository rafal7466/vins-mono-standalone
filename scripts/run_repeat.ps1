<#
.SYNOPSIS
  Runs MH_01 N times in parallel (output\MH01_run<k>) to measure the run-to-run spread caused by
  max_solver_time (Ceres stops after 0.04 s, so the number of iterations depends on the CPU load),
  then evaluates all runs together.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File scripts\run_repeat.ps1 -N 3 -Dataset dataset\MH_01_easy
#>
param(
  [int]$N = 3,
  [string]$Dataset = "dataset\MH_01_easy",
  [string]$Config = "config\euroc_config.yaml",
  [string]$Python = "python"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$exe = Join-Path $root "build\Release\vins_euroc.exe"
if (-not (Test-Path $exe)) { throw "$exe not found - build first with scripts\build.ps1" }
$ds = (Resolve-Path $Dataset).Path
$gt = if (Test-Path "$ds\mav0") { "$ds\mav0\state_groundtruth_estimate0\data.csv" } else { "$ds\state_groundtruth_estimate0\data.csv" }
$procs = @()
for ($k = 1; $k -le $N; $k++) {
  $procs += Start-Process -FilePath $exe -ArgumentList @($Config, "--dataset", $ds, "--out", "output/MH01_run$k", "--log", "2") `
    -WorkingDirectory $root -WindowStyle Hidden -PassThru
}
$procs | Wait-Process
$runs = @(); for ($k = 1; $k -le $N; $k++) { $runs += @("--run", "output/MH01_run$k") }
& $Python scripts/evaluate_euroc.py @runs --gt $gt --out output/evaluation_repeat
