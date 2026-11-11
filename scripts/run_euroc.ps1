<#
.SYNOPSIS
  Runs vins_euroc.exe (VINS-Mono VIO, no loop closure) on one EuRoC sequence and, if Python with
  numpy/pandas/matplotlib is available, evaluates it against the groundtruth.

.PARAMETER Dataset
  EuRoC sequence in ASL format: the folder that contains mav0\ (e.g. dataset\MH_01_easy),
  the mav0 folder itself, or a flattened folder with cam0\ imu0\.
.PARAMETER Name
  Name of the run; outputs go to output\<Name>\. Default: the sequence folder name.
.PARAMETER NoEval
  Skip the Python evaluation.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File scripts\run_euroc.ps1 -Dataset dataset\MH_01_easy
#>
param(
  [Parameter(Mandatory = $true)][string]$Dataset,
  [string]$Name = "",
  [string]$Config = "config\euroc_config.yaml",
  [string]$Python = "python",
  [switch]$NoEval
)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo
$exe = Join-Path $repo "build\Release\vins_euroc.exe"
if (-not (Test-Path $exe)) { throw "$exe not found - build first with scripts\build.ps1" }
if (-not (Test-Path $Dataset)) { throw "Dataset folder '$Dataset' not found (see the Dataset section of the README)" }
$ds = (Resolve-Path $Dataset).Path
if ($Name -eq "") {
  $leaf = Split-Path -Leaf $ds
  if ($leaf -eq "mav0") { $leaf = Split-Path -Leaf (Split-Path -Parent $ds) }
  $Name = $leaf
}
$out = "output/$Name"
Write-Host "$exe $Config --dataset $ds --out $out"
& $exe $Config --dataset $ds --out $out
if ($LASTEXITCODE -ne 0) { throw "vins_euroc.exe exited with code $LASTEXITCODE" }
Get-Content "$out\summary.txt"

if (-not $NoEval) {
  $gt = if (Test-Path "$ds\mav0") { "$ds\mav0\state_groundtruth_estimate0\data.csv" } else { "$ds\state_groundtruth_estimate0\data.csv" }
  if (-not (Test-Path $gt)) { Write-Host "No groundtruth found - evaluation skipped."; exit 0 }
  & $Python scripts/evaluate_euroc.py --run $out --gt $gt --out "$out/evaluation"
  if ($LASTEXITCODE -ne 0) { Write-Warning "Evaluation failed (needs Python with numpy, pandas, matplotlib)" }
}
