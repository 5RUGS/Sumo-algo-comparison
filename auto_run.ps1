param(
  [string[]] $Cities    = @("Fukuoka","Bangkok","Singapore"),
  [string[]] $Densities = @("1","1_5","2","2_5"),
  [string]   $Python    = "python3",
  [string]   $SumoHome  = "/usr/share/sumo"
)

# Anchor to this script's folder (proj)
$Root   = Split-Path -Parent $MyInvocation.MyCommand.Path
$LogDir = Join-Path $Root "logs"
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

# SUMO env
$env:SUMO_HOME  = $SumoHome
$env:PYTHONPATH = "$($env:SUMO_HOME)/tools:$($env:PYTHONPATH)"
Write-Host "SUMO_HOME=$($env:SUMO_HOME)"
Write-Host "PYTHONPATH=$($env:PYTHONPATH)"

function Write-LogSummary {
  param([Parameter(Mandatory)][string] $LogPath)

  if (-not (Test-Path $LogPath)) { return }

  # Find the first "Simulation ended at time:" and keep everything from there to EOF
  $hit = Select-String -Path $LogPath -Pattern '^Simulation ended at time:' | Select-Object -First 1
  if ($null -eq $hit) { return }

  $lines   = Get-Content -LiteralPath $LogPath
  $start   = [int]$hit.LineNumber - 1
  $summary = $lines[$start..($lines.Length - 1)]

  $outPath = [System.IO.Path]::ChangeExtension($LogPath, ".summary.log")
  Set-Content -LiteralPath $outPath -Value $summary
  Write-Host "   -> Wrote summary: $outPath"
}

function Run-One {
  param(
    [Parameter(Mandatory)][string] $CityDir,
    [Parameter(Mandatory)][string] $Density
  )

  # Choose per-density cfg (e.g., osm1ps.sumocfg, osm1_5ps.sumocfg)
  $cfgFile    = "osm{0}ps.sumocfg" -f $Density
  $srcCfgPath = Join-Path $CityDir $cfgFile
  $activeCfg  = Join-Path $CityDir "osm.sumocfg"

  if (-not (Test-Path $srcCfgPath)) {
    Write-Warning "Missing cfg: $srcCfgPath"
    return
  }

  # Put the chosen config in place (no XML editing needed)
  Copy-Item -LiteralPath $srcCfgPath -Destination $activeCfg -Force

  $stamp = (Get-Date).ToString("yyyyMMdd_HHmmss")
  $name  = Split-Path -Leaf $CityDir
  $log1  = Join-Path $LogDir ("{0}_{1}ps_static_{2}.log"   -f $name, $Density, $stamp)
  $log2  = Join-Path $LogDir ("{0}_{1}ps_dijkstra_{2}.log" -f $name, $Density, $stamp)

  Push-Location $CityDir
  try {
    Write-Host "[$name $Density ps] run.py using $cfgFile"
    & $Python (Join-Path $Root "run.py") 2>&1 | Tee-Object -FilePath $log1 -Append | Out-Host

    Write-Host "[$name $Density ps] run_dijkstra.py using $cfgFile"
    & $Python (Join-Path $Root "run_dijkstra.py") 2>&1 | Tee-Object -FilePath $log2 -Append | Out-Host
  } finally {
    Pop-Location
  }

  # Write compact summaries for both logs
  Write-LogSummary -LogPath $log1
  Write-LogSummary -LogPath $log2
}

foreach ($city in $Cities) {
  $cityDir = Join-Path $Root $city
  if (-not (Test-Path $cityDir)) {
    Write-Warning "Missing folder: $cityDir"
    continue
  }
  Write-Host "=== City: $city ==="
  foreach ($d in $Densities) {
    Write-Host "--- Density: $d ps ---"
    Run-One -CityDir $cityDir -Density $d
  }
}

Write-Host "Done. Logs in: $LogDir"
