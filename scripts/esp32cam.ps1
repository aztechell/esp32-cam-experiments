[CmdletBinding()]
param(
  [ValidateSet("ports", "chip", "build", "upload", "monitor")]
  [string]$Action = "build",

  [string]$Port,

  [string[]]$Ports = @("COM7", "COM3", "COM4"),

  [ValidateSet("diagnostic", "web_photo", "mosaic_reader", "mosaic_reader_v2")]
  [string]$Environment = "web_photo"
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$env:PLATFORMIO_CORE_DIR = Join-Path $ProjectRoot ".platformio"

$Python = Join-Path $ProjectRoot ".venv\Scripts\python.exe"
$Pio = Join-Path $ProjectRoot ".venv\Scripts\pio.exe"

$Ports = @(
  foreach ($candidatePort in $Ports) {
    $candidatePort -split "," | ForEach-Object { $_.Trim() } | Where-Object { $_ }
  }
)

function Fail {
  param([string]$Message)

  Write-Host $Message
  exit 1
}

function Assert-File {
  param(
    [string]$Path,
    [string]$Message
  )

  if (-not (Test-Path -LiteralPath $Path)) {
    Fail -Message $Message
  }
}

function Get-OpenableSerialPorts {
  [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
}

function Show-SerialPorts {
  $serialPorts = @(Get-OpenableSerialPorts)
  if ($serialPorts.Count -eq 0) {
    Write-Host "No openable serial ports found."
    return
  }

  Write-Host "Openable serial ports:"
  foreach ($serialPort in $serialPorts) {
    Write-Host "  $serialPort"
  }
}

function Invoke-Esptool {
  param(
    [string]$CandidatePort,
    [string]$Command
  )

  $previousErrorActionPreference = $ErrorActionPreference
  $ErrorActionPreference = "Continue"
  try {
    & $Python -m esptool --port $CandidatePort --baud 115200 $Command |
      ForEach-Object { Write-Host $_ }
    $exitCode = $LASTEXITCODE
  } finally {
    $ErrorActionPreference = $previousErrorActionPreference
  }

  return $exitCode
}

function Test-EspPort {
  param([string]$CandidatePort)

  Write-Host "Checking $CandidatePort..."
  $chipExitCode = Invoke-Esptool -CandidatePort $CandidatePort -Command "chip-id"
  if ($chipExitCode -ne 0) {
    return $false
  }

  $flashExitCode = Invoke-Esptool -CandidatePort $CandidatePort -Command "flash-id"
  if ($flashExitCode -ne 0) {
    Write-Host "$CandidatePort responded to chip-id, but flash-id failed."
  }

  return $true
}

function Find-EspPorts {
  $openablePorts = @(Get-OpenableSerialPorts)
  if ($openablePorts.Count -eq 0) {
    Write-Host "No openable serial ports found."
    return @()
  }

  $foundPorts = @()
  foreach ($candidatePort in $Ports) {
    if ($openablePorts -notcontains $candidatePort) {
      Write-Host "Skipping ${candidatePort}: not openable."
      continue
    }

    if (Test-EspPort -CandidatePort $candidatePort) {
      $foundPorts += $candidatePort
    }
  }

  return $foundPorts
}

function Resolve-EspPort {
  if ($Port) {
    return $Port
  }

  $foundPorts = @(Find-EspPorts)
  if ($foundPorts.Count -eq 0) {
    Fail -Message "No ESP32 bootloader responded. Connect the board, verify the COM port, or use bootloader mode."
  }

  if ($foundPorts.Count -gt 1) {
    Fail -Message "Multiple ESP32 ports responded: $($foundPorts -join ', '). Pass -Port COMx explicitly."
  }

  return $foundPorts[0]
}

function Resolve-MonitorPort {
  if ($Port) {
    return $Port
  }

  $serialPorts = @(Get-OpenableSerialPorts)
  if ($serialPorts.Count -eq 1) {
    return $serialPorts[0]
  }

  Fail -Message "Pass -Port COMx for monitor. Openable ports: $($serialPorts -join ', ')"
}

Assert-File -Path $Python -Message "Missing .venv Python. Create it and install PlatformIO/esptool first."
Assert-File -Path $Pio -Message "Missing PlatformIO CLI. Install it with: .\.venv\Scripts\python.exe -m pip install platformio esptool"

switch ($Action) {
  "ports" {
    Show-SerialPorts
  }
  "chip" {
    $foundPorts = @(Find-EspPorts)
    if ($foundPorts.Count -eq 0) {
      Fail -Message "No ESP32 bootloader responded on: $($Ports -join ', ')"
    }

    Write-Host "ESP32 responded on: $($foundPorts -join ', ')"
  }
  "build" {
    & $Pio run -e $Environment
    exit $LASTEXITCODE
  }
  "upload" {
    $targetPort = Resolve-EspPort
    Write-Host "Uploading $Environment to $targetPort..."
    & $Pio run -e $Environment -t upload --upload-port $targetPort
    exit $LASTEXITCODE
  }
  "monitor" {
    $targetPort = Resolve-MonitorPort
    & $Pio device monitor --port $targetPort --baud 115200 --rts 0 --dtr 0
    exit $LASTEXITCODE
  }
}
