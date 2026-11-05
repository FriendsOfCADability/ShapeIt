<#
.SYNOPSIS
    Baut ShapeIt.Avalonia fuer Linux x64 und erstellt ein ZIP fuer das Linux-Packaging.

.DESCRIPTION
    Liest die Version aus version.txt im Repo-Root, fuehrt dotnet publish aus
    und legt shapeit-linux-x64-VERSION.zip im publish-Verzeichnis ab.

.EXAMPLE
    # Version aus version.txt lesen:
    .\1-build.ps1

    # Version explizit angeben:
    .\1-build.ps1 -Version "1.2.0"
#>
param(
    [string]$Version = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# Pfade
$ScriptDir   = $PSScriptRoot
$RepoRoot    = Resolve-Path (Join-Path $ScriptDir "..\..")
$VersionFile = Join-Path $RepoRoot "version.txt"
$CsprojFile  = Join-Path $RepoRoot "ShapeIt.Avalonia\ShapeIt.Avalonia.csproj"
$PublishDir  = Join-Path $RepoRoot "publish\linux-x64"
$OutDir      = Join-Path $RepoRoot "publish"

# Version ermitteln
if (-not $Version) {
    if (-not (Test-Path $VersionFile)) {
        Write-Error "version.txt nicht gefunden: $VersionFile"
        exit 1
    }
    $Version = (Get-Content $VersionFile -Raw).Trim()
}

if ($Version -notmatch '^\d+\.\d+\.\d+$') {
    Write-Error "Ungueltiges Versionsformat '$Version' - erwartet: MAJOR.MINOR.PATCH (z.B. 1.2.3)"
    exit 1
}

Write-Host ""
Write-Host "=====================================================" -ForegroundColor Cyan
Write-Host "  ShapeIt $Version - Linux x64 Build"               -ForegroundColor Cyan
Write-Host "=====================================================" -ForegroundColor Cyan
Write-Host ""

# dotnet publish
Write-Host "Schritt 1/2: dotnet publish..." -ForegroundColor Yellow

& dotnet publish $CsprojFile `
    -r linux-x64 `
    --self-contained true `
    -c Release `
    -o $PublishDir `
    -p:AssemblyVersion=$Version `
    -p:FileVersion=$Version

if ($LASTEXITCODE -ne 0) {
    Write-Error "dotnet publish fehlgeschlagen (Exit-Code $LASTEXITCODE)"
    exit 1
}

# ZIP erstellen
Write-Host ""
Write-Host "Schritt 2/2: ZIP erstellen..." -ForegroundColor Yellow

$ZipName = "shapeit-linux-x64-$Version.zip"
$ZipPath = Join-Path $OutDir $ZipName

Remove-Item $ZipPath -ErrorAction SilentlyContinue

# Build the ZIP manually instead of using Compress-Archive: Windows PowerShell 5.1
# writes entry names with backslashes, which unzip on Linux rejects
# ("appears to use backslashes as path separators"). Entries need forward slashes.
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem

$PublishRoot = (Resolve-Path $PublishDir).Path.TrimEnd('\') + '\'
$Zip = [System.IO.Compression.ZipFile]::Open($ZipPath, [System.IO.Compression.ZipArchiveMode]::Create)
try {
    Get-ChildItem -Path $PublishDir -Recurse -File | ForEach-Object {
        $EntryName = $_.FullName.Substring($PublishRoot.Length).Replace('\', '/')
        [void][System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile(
            $Zip, $_.FullName, $EntryName, [System.IO.Compression.CompressionLevel]::Optimal)
    }
}
finally {
    $Zip.Dispose()
}

$SizeMB = [math]::Round((Get-Item $ZipPath).Length / 1MB, 1)

Write-Host ""
Write-Host "=====================================================" -ForegroundColor Green
Write-Host "  Fertig!"                                            -ForegroundColor Green
Write-Host "=====================================================" -ForegroundColor Green
Write-Host ""
Write-Host "  ZIP:  $ZipPath  ($SizeMB MB)" -ForegroundColor White
Write-Host ""
Write-Host "Naechster Schritt:" -ForegroundColor Yellow
Write-Host "  ZIP auf den Linux-Rechner kopieren (USB-Stick oder SCP), dann:" -ForegroundColor White
Write-Host "  bash 2-package.sh $Version $ZipName" -ForegroundColor Cyan
Write-Host ""
