# One-click DSP/SIMD micro-benchmark + update chipset report snippet
param(
    [int]$Iters = 80,
    [switch]$SkipBuild
)

$ErrorActionPreference = "Continue"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$DockerDir = Join-Path $Root "VisionPilot\docker"
$OutDir = Join-Path $Root "platforms\benchmarks"
$OutLog = Join-Path $OutDir "last_perf_bench.txt"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

function Assert-Ok([string]$What) {
    if ($LASTEXITCODE -ne 0) { throw "$What failed (exit $LASTEXITCODE)" }
}

Write-Host "==> Docker"
docker info 2>$null | Out-Null
Assert-Ok "Docker"

Push-Location $DockerDir
try {
    if (-not $SkipBuild) {
        Write-Host "==> Building visionpilot:cpu (includes vp_perf_bench)"
        docker build -t visionpilot:cpu -f Dockerfile.cpu --build-arg ENABLE_ROS2=OFF ..
        Assert-Ok "docker build"
    }

    Write-Host "==> Running vp_perf_bench"
    docker run --rm --entrypoint /usr/bin/vp_perf_bench visionpilot:cpu --iters $Iters --w 1024 --h 512 2>&1 |
        Tee-Object -FilePath $OutLog
    Assert-Ok "vp_perf_bench"
} finally {
    Pop-Location
}

Write-Host ""
Write-Host "Saved: $OutLog"
Write-Host "See also: docs/BENCHMARK_CHIPSET_COMPARISON.md"
