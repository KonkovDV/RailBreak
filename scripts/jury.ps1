# Same scenarios on Windows, Linux and macOS, via Docker Desktop.
#   .\scripts\jury.ps1 smoke -Bag D:\bag -Msgs D:\tram_vehicle_msgs
#   .\scripts\jury.ps1 core
param(
    [Parameter(Position = 0)]
    [ValidateSet('play', 'smoke', 'clock', 'fast', 'frame', 'no-gnss', 'no-assets', 'arc', 'record', 'core')]
    [string]$Scenario = 'play',
    [string]$Bag = '',
    [string]$Msgs = '',
    [string]$InitialS = '0'
)

$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)

function Invoke-Core {
    cmake -S (Join-Path $Root 'railbreak_backup_odometry/tools') -B (Join-Path $Root 'build/rbo')
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    cmake --build (Join-Path $Root 'build/rbo') --config Release --parallel
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    ctest --test-dir (Join-Path $Root 'build/rbo') -C Release --output-on-failure
    exit $LASTEXITCODE
}

if ($Scenario -eq 'core') {
    Invoke-Core
    exit 0
}

if (-not $Msgs) {
    foreach ($candidate in @('tram_vehicle_msgs', 'files/tram_vehicle_msgs')) {
        $path = Join-Path $Root $candidate
        if (Test-Path (Join-Path $path 'package.xml')) { $Msgs = $path; break }
    }
}
if (-not $Msgs -or -not (Test-Path (Join-Path $Msgs 'package.xml'))) {
    throw 'Pass -Msgs, the tram_vehicle_msgs directory (package.xml inside)'
}
if (-not $Bag -or -not (Test-Path (Join-Path $Bag 'metadata.yaml'))) {
    throw 'Pass -Bag, a rosbag2 directory that contains metadata.yaml'
}

$env:MSGS_DIR = (Resolve-Path $Msgs).Path.Replace('\', '/')
$env:BAG = (Resolve-Path $Bag).Path.Replace('\', '/')
$env:OUT = (Join-Path $Root 'jury_out').Replace('\', '/')
$env:RATE = '1'
$env:CLOCK = '0'
$env:OUTPUT_FRAME = 'mgrs'
$env:ASSETS_DIR = ''
$env:TOPICS = ''
$env:GNSS_WINDOW = '3.0'
$env:GNSS_WAIT = ''
$env:INITIAL_S = ''
$env:DURATION = ''
$env:RECORD = '0'
$env:SCORE = '0'
$env:REQUIRE_POSITION = '1'
$env:SCORE_MODE = 'acceptance'

switch ($Scenario) {
    'smoke' { $env:DURATION = '25' }
    'clock' { $env:CLOCK = '1' }
    'fast' { $env:RATE = '10' }
    'frame' { $env:OUTPUT_FRAME = 'mkrs_start' }
    'no-gnss' {
        $env:GNSS_WAIT = '2'
        $env:TOPICS = '/vehicle/front_bogie_velocity /vehicle/rear_bogie_velocity /vehicle/driver_position_cmd'
    }
    'no-assets' { $env:ASSETS_DIR = '/nonexistent'; $env:REQUIRE_POSITION = '0' }
    'arc' {
        $env:GNSS_WAIT = '2'
        $env:INITIAL_S = $InitialS
        $env:TOPICS = '/vehicle/front_bogie_velocity /vehicle/rear_bogie_velocity /vehicle/driver_position_cmd'
    }
    'record' { $env:RECORD = '1'; $env:SCORE = '1'; $env:SCORE_MODE = 'acceptance' }
}

Push-Location $Root
try {
    docker compose -f docker-compose.jury.yml run -T --rm --build jury
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
