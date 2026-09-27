$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

function Invoke-RepoCommand {
    param(
        [Parameter(Mandatory = $true)]
        [string] $Description,

        [Parameter(Mandatory = $true)]
        [scriptblock] $Command
    )

    Write-Host ""
    Write-Host "==> $Description"
    & $Command
    if ($LASTEXITCODE -ne 0) {
        throw "$Description failed with exit code $LASTEXITCODE"
    }
}

$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

Write-Host "Java version:"
java -version
if ($LASTEXITCODE -ne 0) {
    throw "java -version failed with exit code $LASTEXITCODE"
}

Write-Host ""
Write-Host "Repository location:"
Write-Host $RepoRoot

Invoke-RepoCommand "Running Gradle clean test" { .\gradlew.bat clean test }
Invoke-RepoCommand "Running Gradle build" { .\gradlew.bat build }

Write-Host ""
Write-Host "PASS: Windows validation build and tests completed successfully."
