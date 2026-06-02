<#
.SYNOPSIS
    Erstellt den Windows-Installer (Setup.exe) fuer ShapeIt mit Inno Setup.

.DESCRIPTION
    Gegenstueck zum Linux 2-package.sh (.deb): erzeugt aus der self-contained
    Publish-Ausgabe einen klassischen Windows-Installer mit Setup-Assistent,
    Startmenue-Eintrag, optionaler Desktop-Verknuepfung und Uninstaller.

    Voraussetzung: Inno Setup 6 (ISCC.exe). Falls nicht gefunden, gibt das Skript
    den Installationshinweis aus (winget install JRSoftware.InnoSetup).

    Ablauf:
      1. Version aus version.txt (Repo-Root) lesen.
      2. Bei -Build (oder fehlendem Publish-Ordner) zuerst 1-build.ps1 ausfuehren.
      3. ISCC.exe mit den passenden /D-Defines aufrufen.
      4. Ergebnis: publish\shapeit-setup-VERSION.exe

.EXAMPLE
    # Installer aus vorhandenem Publish-Ordner bauen:
    .\2-installer.ps1

    # Vorher frisch publishen, dann Installer bauen:
    .\2-installer.ps1 -Build

    # Version explizit setzen:
    .\2-installer.ps1 -Version "1.2.0"
#>
param(
    [string]$Version = "",
    [switch]$Build
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# Pfade
$ScriptDir   = $PSScriptRoot
$RepoRoot    = Resolve-Path (Join-Path $ScriptDir "..\..")
$VersionFile = Join-Path $RepoRoot "version.txt"
$PublishDir  = Join-Path $RepoRoot "publish\win-x64"
$OutDir      = Join-Path $RepoRoot "publish"
$IssFile     = Join-Path $ScriptDir "installer\ShapeIt.iss"
$IconFile    = Join-Path $RepoRoot "ShapeIt.Avalonia\Logo.ico"
$BuildScript = Join-Path $ScriptDir "1-build.ps1"

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
Write-Host "  ShapeIt $Version - Windows-Installer (Inno Setup)" -ForegroundColor Cyan
Write-Host "=====================================================" -ForegroundColor Cyan
Write-Host ""

# ── ISCC.exe finden ──────────────────────────────────────────────────────────
$IsccCandidates = @(
    "C:\Program Files (x86)\Inno Setup 6\ISCC.exe",
    "C:\Program Files\Inno Setup 6\ISCC.exe",
    (Join-Path $env:LOCALAPPDATA "Programs\Inno Setup 6\ISCC.exe")
)
$Iscc = $IsccCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $Iscc) {
    $cmd = Get-Command iscc -ErrorAction SilentlyContinue
    if ($cmd) { $Iscc = $cmd.Source }
}
if (-not $Iscc) {
    Write-Error @"
Inno Setup (ISCC.exe) nicht gefunden.
Installieren mit:
    winget install JRSoftware.InnoSetup
oder herunterladen von https://jrsoftware.org/isdl.php
"@
    exit 1
}
Write-Host "Inno Setup: $Iscc" -ForegroundColor DarkGray

# ── Schritt 1: ggf. publishen ────────────────────────────────────────────────
if ($Build -or -not (Test-Path (Join-Path $PublishDir "ShapeIt.exe"))) {
    Write-Host ""
    Write-Host "Schritt 1/2: Publish (1-build.ps1)..." -ForegroundColor Yellow
    & powershell -ExecutionPolicy Bypass -File $BuildScript -Version $Version
    if ($LASTEXITCODE -ne 0) {
        Write-Error "1-build.ps1 fehlgeschlagen (Exit-Code $LASTEXITCODE)"
        exit 1
    }
} else {
    Write-Host "Schritt 1/2: Vorhandenen Publish-Ordner verwenden." -ForegroundColor DarkGray
    Write-Host "             ($PublishDir)" -ForegroundColor DarkGray
    Write-Host "             Fuer einen frischen Build: -Build verwenden." -ForegroundColor DarkGray
}

if (-not (Test-Path (Join-Path $PublishDir "ShapeIt.exe"))) {
    Write-Error "ShapeIt.exe nicht im Publish-Ordner gefunden: $PublishDir"
    exit 1
}

# ── Schritt 2: Installer kompilieren ─────────────────────────────────────────
Write-Host ""
Write-Host "Schritt 2/2: Installer kompilieren..." -ForegroundColor Yellow

& $Iscc `
    "/DAppVersion=$Version" `
    "/DPublishDir=$PublishDir" `
    "/DOutputDir=$OutDir" `
    "/DIconFile=$IconFile" `
    $IssFile

if ($LASTEXITCODE -ne 0) {
    Write-Error "ISCC.exe fehlgeschlagen (Exit-Code $LASTEXITCODE)"
    exit 1
}

$SetupPath = Join-Path $OutDir "shapeit-setup-$Version.exe"
$SizeMB = if (Test-Path $SetupPath) { [math]::Round((Get-Item $SetupPath).Length / 1MB, 1) } else { "?" }

Write-Host ""
Write-Host "=====================================================" -ForegroundColor Green
Write-Host "  Fertig!"                                            -ForegroundColor Green
Write-Host "=====================================================" -ForegroundColor Green
Write-Host ""
Write-Host "  Setup:  $SetupPath  ($SizeMB MB)" -ForegroundColor White
Write-Host ""
Write-Host "Anwender: Setup.exe ausfuehren -> Assistent -> ShapeIt startet" -ForegroundColor White
Write-Host "  (Standard: Installation pro Benutzer ohne Administratorrechte)" -ForegroundColor White
Write-Host ""
