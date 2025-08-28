param(
  [string[]] $Cities    = @("Fukuoka","Bangkok","Singapore"),
  [string[]] $Densities = @("1","1_5","2","2_5"),
  [string]   $Python    = "python3",
  [string]   $SumoHome  = "/usr/share/sumo",
  [int]      $TimeoutSecPerRun = 7200     # 2h safety timeout per run
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

function Stop-LeftoverSumo {
  # Kill any orphaned SUMO processes that might keep the TraCI port busy
  $left = Get-Process -Name "sumo","sumo-gui" -ErrorAction SilentlyContinue
  if ($left) {
    Write-Warning ("Killing {0} leftover SUMO process(es)..." -f $left.Count)
    $left | Stop-Process -Force
    Start-Sleep -Seconds 1
  }
}

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

function Invoke-ProcessWithTimeout {
  param(
    [Parameter(Mandatory)][string] $FilePath,
    [string[]] $ArgumentList = @(),
    [Parameter(Mandatory)][string] $StdOutPath,
    [string] $StdErrPath,
    [int] $TimeoutSec = 7200
  )

  # Ensure log folder exists
  $outDir = Split-Path -Parent $StdOutPath
  if ($outDir) { New-Item -ItemType Directory -Force -Path $outDir | Out-Null }

  # If stderr path not given or equals stdout path, use a temp stderr file and merge later
  $useTempErr = $false
  if ([string]::IsNullOrEmpty($StdErrPath) -or ($StdErrPath -eq $StdOutPath)) {
    $StdErrPath = [System.IO.Path]::ChangeExtension($StdOutPath, ".stderr.tmp")
    $useTempErr = $true
  }

  # Clear/initialize the target files so we don't append to old runs
  New-Item -ItemType File -Force -Path $StdOutPath | Out-Null
  New-Item -ItemType File -Force -Path $StdErrPath | Out-Null

  # Start process (portable params only)
  $psi = @{
    FilePath               = $FilePath
    ArgumentList           = $ArgumentList
    RedirectStandardOutput = $StdOutPath
    RedirectStandardError  = $StdErrPath
    PassThru               = $true
  }
  $proc = Start-Process @psi

  # Wait up to timeout; if still running, kill it
  $finished = $true
  try {
    $finished = $proc.WaitForExit($TimeoutSec * 1000)
  } catch {
    $finished = $false
  }

  if (-not $finished) {
    Write-Warning "Process '$FilePath' exceeded $TimeoutSec s; killing."
    try { Stop-Process -Id $proc.Id -Force } catch {}
  }

  # If we used a temp stderr file, append it into stdout and then remove it
  if ($useTempErr -and (Test-Path -LiteralPath $StdErrPath)) {
    try {
      Add-Content -LiteralPath $StdOutPath -Value "`n--- STDERR ---`n"
      Get-Content -LiteralPath $StdErrPath | Add-Content -LiteralPath $StdOutPath
    } finally {
      Remove-Item -LiteralPath $StdErrPath -Force -ErrorAction SilentlyContinue
    }
  }

  # Return success/failure
  if (-not $finished) { return $false }
  try {
    return ($proc.ExitCode -eq 0)
  } catch {
    return $false
  }
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
    # PRE: ensure no stuck SUMO from previous run
    Stop-LeftoverSumo

    Write-Host "[$name $Density ps] run.py using $cfgFile"
    $ok1 = Invoke-ProcessWithTimeout `
      -FilePath $Python `
      -ArgumentList @((Join-Path $Root "run.py")) `
      -StdOutPath $log1 `
      -TimeoutSec $TimeoutSecPerRun

    # POST: clean any leftover SUMO
    Stop-LeftoverSumo

    Write-Host "[$name $Density ps] run_dijkstra.py using $cfgFile"
    $ok2 = Invoke-ProcessWithTimeout `
      -FilePath $Python `
      -ArgumentList @((Join-Path $Root "run_dijkstra.py")) `
      -StdOutPath $log2 `
      -TimeoutSec $TimeoutSecPerRun

    # POST: clean again for next density
    Stop-LeftoverSumo
  } finally {
    Pop-Location
  }

  # Write compact summaries for both logs (works even if a run timed out)
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
