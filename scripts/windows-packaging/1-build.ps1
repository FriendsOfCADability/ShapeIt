<#
.SYNOPSIS
    Baut ShapeIt.Avalonia als self-contained Anwendung fuer Windows x64 und
    erstellt ein ZIP zum Download. Die ausfuehrbare Datei heisst ShapeIt.exe.

.DESCRIPTION
    Gegenstueck zum Linux-Build (scripts\linux-packaging\1-build.ps1): es wird
    DASSELBE Projekt (ShapeIt.Avalonia) und DIESELBE version.txt (Repo-Root)
    verwendet - eine einzige Codebasis fuer Windows und Linux.

    Fuehrt 'dotnet publish' fuer ShapeIt.Avalonia aus (Release, win-x64,
    self-contained). Der Anwender braucht KEIN installiertes .NET: ZIP entpacken,
    ShapeIt.exe starten - fertig. Das ist der robuste Ersatz fuer den bisherigen
    ClickOnce-Fallback-ZIP.

    Der Ausgabename ShapeIt.exe kommt aus <AssemblyName>ShapeIt</AssemblyName>
    in ShapeIt.Avalonia.csproj (gilt fuer Windows und Linux gleichermassen).

    Bewusst KEIN Single-File-Build: CADability/OpenGL bringt native DLLs mit, die
    Single-File zur Laufzeit nach %TEMP% entpacken muesste - genau die Reibung,
    die wir vermeiden wollen. Ein self-contained Ordner-ZIP ist am robustesten
    und entspricht dem Linux-Build.

.EXAMPLE
    # Version aus version.txt lesen:
    .\1-build.ps1

    # Version explizit angeben:
    .\1-build.ps1 -Version "1.2.0"

    # Nur das ZIP aus einem vorhandenen Publish-Ordner neu erstellen:
    .\1-build.ps1 -SkipBuild
#>
param(
    [string]$Version = "",
    [switch]$SkipBuild
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# Pfade
$ScriptDir   = $PSScriptRoot
$RepoRoot    = Resolve-Path (Join-Path $ScriptDir "..\..")
$CsprojFile  = Join-Path $RepoRoot "ShapeIt.Avalonia\ShapeIt.Avalonia.csproj"
$VersionFile = Join-Path $RepoRoot "version.txt"
$PublishDir  = Join-Path $RepoRoot "publish\win-x64"
$OutDir      = Join-Path $RepoRoot "publish"

if (-not (Test-Path $CsprojFile)) {
    Write-Error "Projektdatei nicht gefunden: $CsprojFile"
    exit 1
}

# ── Version ermitteln ────────────────────────────────────────────────────────
if (-not $Version) {
    if (-not (Test-Path $VersionFile)) {
        Write-Error "version.txt nicht gefunden: $VersionFile"
        exit 1
    }
    $Version = (Get-Content $VersionFile -Raw).Trim()
}

if ($Version -notmatch '^\d+\.\d+\.\d+(\.\d+)?$') {
    Write-Error "Ungueltiges Versionsformat '$Version' - erwartet z.B. 1.2.3 oder 1.2.3.4"
    exit 1
}

Write-Host ""
Write-Host "=====================================================" -ForegroundColor Cyan
Write-Host "  ShapeIt $Version - Windows x64 Build (self-contained)" -ForegroundColor Cyan
Write-Host "=====================================================" -ForegroundColor Cyan
Write-Host ""

# ── Schritt 1: dotnet publish ────────────────────────────────────────────────
if (-not $SkipBuild) {
    Write-Host "Schritt 1/2: dotnet publish..." -ForegroundColor Yellow

    # Publish-Ordner leeren, damit keine alten Dateien zurueckbleiben
    if (Test-Path $PublishDir) {
        Remove-Item $PublishDir -Recurse -Force
    }

    & dotnet publish $CsprojFile `
        -r win-x64 `
        --self-contained true `
        -c Release `
        -o $PublishDir `
        -p:AssemblyVersion=$Version `
        -p:FileVersion=$Version

    if ($LASTEXITCODE -ne 0) {
        Write-Error "dotnet publish fehlgeschlagen (Exit-Code $LASTEXITCODE)"
        exit 1
    }
} else {
    Write-Host "Schritt 1/2: Build uebersprungen (-SkipBuild)." -ForegroundColor DarkGray
    if (-not (Test-Path $PublishDir)) {
        Write-Error "Publish-Ordner nicht gefunden: $PublishDir - bitte ohne -SkipBuild ausfuehren."
        exit 1
    }
}

# Sicherstellen, dass die Haupt-EXE korrekt benannt ist
$ExePath = Join-Path $PublishDir "ShapeIt.exe"
if (-not (Test-Path $ExePath)) {
    Write-Error "ShapeIt.exe nicht im Publish-Ordner gefunden: $ExePath`n(Steht <AssemblyName>ShapeIt</AssemblyName> in ShapeIt.Avalonia.csproj?)"
    exit 1
}

# ── Schritt 2: ZIP erstellen ─────────────────────────────────────────────────
Write-Host ""
Write-Host "Schritt 2/2: ZIP erstellen..." -ForegroundColor Yellow

$ZipName = "shapeit-win-x64-$Version.zip"
$ZipPath = Join-Path $OutDir $ZipName

Remove-Item $ZipPath -ErrorAction SilentlyContinue
Compress-Archive -Path "$PublishDir\*" -DestinationPath $ZipPath

$SizeMB = [math]::Round((Get-Item $ZipPath).Length / 1MB, 1)

# latest.txt fuer die In-App-Update-Pruefung (UpdateChecker.cs) miterzeugen.
# Diese Datei neben die Downloads auf den Server legen; ShapeIt liest sie beim Start.
$LatestTxt = Join-Path $OutDir "latest.txt"
Set-Content -Path $LatestTxt -Value $Version -NoNewline -Encoding ASCII

Write-Host ""
Write-Host "=====================================================" -ForegroundColor Green
Write-Host "  Fertig!"                                            -ForegroundColor Green
Write-Host "=====================================================" -ForegroundColor Green
Write-Host ""
Write-Host "  ZIP:         $ZipPath  ($SizeMB MB)" -ForegroundColor White
Write-Host "  latest.txt:  $LatestTxt  (Inhalt: $Version)" -ForegroundColor White
Write-Host ""
Write-Host "Anwender-Anleitung:" -ForegroundColor Yellow
Write-Host "  1. ZIP herunterladen und entpacken (Rechtsklick > Alle extrahieren)" -ForegroundColor White
Write-Host "  2. ShapeIt.exe starten - kein .NET noetig" -ForegroundColor White
Write-Host ""
