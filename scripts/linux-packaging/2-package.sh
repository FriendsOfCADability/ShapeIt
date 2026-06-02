#!/bin/bash
# =============================================================================
#  2-package.sh  –  .deb-Paket erstellen und APT-Repository aktualisieren
# =============================================================================
#
#  Aufruf:
#    bash 2-package.sh VERSION ZIP-DATEI [REPO-VERZEICHNIS]
#
#  Beispiele:
#    bash 2-package.sh 1.0.0 shapeit-linux-x64-1.0.0.zip
#    bash 2-package.sh 1.0.0 shapeit-linux-x64-1.0.0.zip ~/shapeit-apt-repo
#
#  Was dieses Skript tut:
#    1. ZIP entpacken  →  Binaries nach /opt/shapeit/
#    2. .deb-Paket bauen  (DEBIAN/control + Dateistruktur)
#    3. APT-Repository aktualisieren  (Packages, Packages.gz, Release)
#    4. GPG-Signierung  (wenn ein Schlüssel vorhanden ist)
#    5. Upload-Befehl anzeigen
# =============================================================================

set -euo pipefail

# ── Parameter ──────────────────────────────────────────────────────────────────
VERSION="${1:-}"
ZIP_FILE="${2:-}"
REPO_DIR="${3:-$HOME/shapeit-apt-repo}"

if [[ -z "$VERSION" || -z "$ZIP_FILE" ]]; then
    echo "Fehler: Aufruf: bash 2-package.sh VERSION ZIP-DATEI [REPO-VERZEICHNIS]"
    echo "  Beispiel: bash 2-package.sh 1.0.0 shapeit-linux-x64-1.0.0.zip"
    exit 1
fi

if [[ ! -f "$ZIP_FILE" ]]; then
    echo "Fehler: ZIP-Datei nicht gefunden: $ZIP_FILE"
    exit 1
fi

# ── Konfiguration ──────────────────────────────────────────────────────────────
PKG_NAME="shapeit"
MAINTAINER="Gerhard Hofmann <shapeit@cadability.de>"
DESCRIPTION_SHORT="ShapeIt - 3D CAD Anwendung"
DESCRIPTION_LONG=" Plattformunabhängige 3D CAD Anwendung auf Basis von Avalonia und OpenGL."
HOMEPAGE="https://cadability.de/ShapeIt"
SECTION="graphics"
DEPENDS="libgl1, libx11-6, libfontconfig1"
GPG_KEY_ID="ShapeIt Repository"   # Name des GPG-Schlüssels (aus 3-setup-gpg.sh)

DEB_NAME="${PKG_NAME}_${VERSION}_amd64.deb"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "====================================================="
echo "  ShapeIt $VERSION  –  .deb-Paket + APT-Repo"
echo "====================================================="
echo ""

# ── Schritt 1: ZIP entpacken ──────────────────────────────────────────────────
echo "[ 1/5 ] ZIP entpacken..."
UNZIP_DIR="$WORK_DIR/publish"
mkdir -p "$UNZIP_DIR"
unzip -q "$ZIP_FILE" -d "$UNZIP_DIR"

# ── Schritt 2: .deb Verzeichnisstruktur aufbauen ──────────────────────────────
echo "[ 2/5 ] .deb-Struktur aufbauen..."

DEB_STAGE="$WORK_DIR/deb-stage"

# Anwendungsverzeichnis: /opt/shapeit/
APP_DIR="$DEB_STAGE/opt/shapeit"
mkdir -p "$APP_DIR"
cp -a "$UNZIP_DIR"/. "$APP_DIR/"
chmod +x "$APP_DIR/ShapeIt"

# Wrapper-Skript: /usr/local/bin/shapeit  (damit der Befehl "shapeit" funktioniert)
BIN_DIR="$DEB_STAGE/usr/local/bin"
mkdir -p "$BIN_DIR"
cat > "$BIN_DIR/shapeit" << 'EOF'
#!/bin/bash
exec /opt/shapeit/ShapeIt "$@"
EOF
chmod +x "$BIN_DIR/shapeit"

# Icon installieren: in den freedesktop-Icon-Pfad legen, damit das Anwendungsmenü
# und (unter Wayland) Titel-/Taskleiste das in der .desktop-Datei referenzierte
# "Icon=shapeit" auflösen können. Quelle ist Logo.png aus dem Publish-Output
# (per <None CopyToOutputDirectory> in ShapeIt.Avalonia.csproj mitkopiert).
ICON_SRC="$UNZIP_DIR/Logo.png"
if [[ -f "$ICON_SRC" ]]; then
    # hicolor/256x256: bevorzugter, skalierbarer Pfad für Icon-Themes
    ICON_DIR="$DEB_STAGE/usr/share/icons/hicolor/256x256/apps"
    mkdir -p "$ICON_DIR"
    cp "$ICON_SRC" "$ICON_DIR/shapeit.png"
    # pixmaps: einfacher Fallback-Pfad (von praktisch allen Desktops unterstützt)
    PIXMAP_DIR="$DEB_STAGE/usr/share/pixmaps"
    mkdir -p "$PIXMAP_DIR"
    cp "$ICON_SRC" "$PIXMAP_DIR/shapeit.png"
else
    echo "        WARNUNG: Logo.png nicht im Publish-Output gefunden – Icon wird nicht installiert."
    echo "                 (ShapeIt.Avalonia.csproj muss Logo.png in den Output kopieren.)"
fi

# Desktop-Eintrag: erscheint im Anwendungsmenü von Linux Mint
DESKTOP_DIR="$DEB_STAGE/usr/share/applications"
mkdir -p "$DESKTOP_DIR"
cat > "$DESKTOP_DIR/shapeit.desktop" << EOF
[Desktop Entry]
Name=ShapeIt
Comment=3D CAD Anwendung
Exec=/usr/local/bin/shapeit
Icon=shapeit
Terminal=false
Type=Application
Categories=Graphics;3DGraphics;Engineering;
StartupWMClass=ShapeIt
EOF

# DEBIAN/control  –  Paket-Metadaten
DEBIAN_DIR="$DEB_STAGE/DEBIAN"
mkdir -p "$DEBIAN_DIR"

INSTALLED_SIZE_KB=$(du -sk "$APP_DIR" | cut -f1)

cat > "$DEBIAN_DIR/control" << EOF
Package: $PKG_NAME
Version: $VERSION
Architecture: amd64
Maintainer: $MAINTAINER
Installed-Size: $INSTALLED_SIZE_KB
Depends: $DEPENDS
Section: $SECTION
Priority: optional
Homepage: $HOMEPAGE
Description: $DESCRIPTION_SHORT
$DESCRIPTION_LONG
EOF

# postinst: wird nach der Installation ausgeführt
cat > "$DEBIAN_DIR/postinst" << 'EOF'
#!/bin/bash
chmod +x /opt/shapeit/ShapeIt
chmod +x /usr/local/bin/shapeit
# Desktop-Datenbank aktualisieren (damit ShapeIt im Menü erscheint)
update-desktop-database /usr/share/applications 2>/dev/null || true
# Icon-Cache aktualisieren (damit das Icon sofort im Menü angezeigt wird)
gtk-update-icon-cache -f -t /usr/share/icons/hicolor 2>/dev/null || true
EOF
chmod 755 "$DEBIAN_DIR/postinst"

# prerm: wird vor dem Deinstallieren ausgeführt
cat > "$DEBIAN_DIR/prerm" << 'EOF'
#!/bin/bash
update-desktop-database /usr/share/applications 2>/dev/null || true
gtk-update-icon-cache -f -t /usr/share/icons/hicolor 2>/dev/null || true
EOF
chmod 755 "$DEBIAN_DIR/prerm"

# ── Schritt 3: .deb-Paket bauen ───────────────────────────────────────────────
echo "[ 3/5 ] .deb-Paket bauen..."
DEB_OUTPUT="$WORK_DIR/$DEB_NAME"
dpkg-deb --root-owner-group --build "$DEB_STAGE" "$DEB_OUTPUT"
echo "        Erstellt: $DEB_NAME ($(du -sh "$DEB_OUTPUT" | cut -f1))"

# ── Schritt 4: APT-Repository aktualisieren ───────────────────────────────────
echo "[ 4/5 ] APT-Repository aktualisieren..."

# Verzeichnisstruktur anlegen
POOL_DIR="$REPO_DIR/pool/main/s/shapeit"
DISTS_DIR="$REPO_DIR/dists/stable/main/binary-amd64"
mkdir -p "$POOL_DIR"
mkdir -p "$DISTS_DIR"

# .deb in den Pool kopieren
cp "$DEB_OUTPUT" "$POOL_DIR/$DEB_NAME"

# Packages-Datei erstellen (listet alle verfügbaren Pakete)
PACKAGES_FILE="$DISTS_DIR/Packages"
> "$PACKAGES_FILE"   # Datei leeren (für zukünftige Mehrfach-Pakete ggf. anpassen)

for DEB in "$POOL_DIR"/*.deb; do
    [[ -f "$DEB" ]] || continue
    REL_PATH="pool/main/s/shapeit/$(basename "$DEB")"
    SIZE=$(stat -c %s "$DEB")
    MD5=$(md5sum "$DEB" | cut -d' ' -f1)
    SHA1=$(sha1sum "$DEB" | cut -d' ' -f1)
    SHA256=$(sha256sum "$DEB" | cut -d' ' -f1)

    # Metadaten aus dem .deb lesen
    DEB_CONTROL=$(dpkg-deb -f "$DEB")

    cat >> "$PACKAGES_FILE" << EOF
$(echo "$DEB_CONTROL")
Filename: $REL_PATH
Size: $SIZE
MD5sum: $MD5
SHA1: $SHA1
SHA256: $SHA256

EOF
done

# Packages.gz erzeugen
gzip -9 -c "$PACKAGES_FILE" > "$DISTS_DIR/Packages.gz"

# Release-Datei erzeugen (enthält Prüfsummen der Packages-Datei)
PKGS_SIZE=$(stat -c %s "$PACKAGES_FILE")
PKGS_GZ_SIZE=$(stat -c %s "$DISTS_DIR/Packages.gz")
PKGS_MD5=$(md5sum "$PACKAGES_FILE" | cut -d' ' -f1)
PKGS_SHA256=$(sha256sum "$PACKAGES_FILE" | cut -d' ' -f1)
PKGS_GZ_MD5=$(md5sum "$DISTS_DIR/Packages.gz" | cut -d' ' -f1)
PKGS_GZ_SHA256=$(sha256sum "$DISTS_DIR/Packages.gz" | cut -d' ' -f1)

RELEASE_FILE="$REPO_DIR/dists/stable/Release"
cat > "$RELEASE_FILE" << EOF
Origin: ShapeIt
Label: ShapeIt
Suite: stable
Codename: stable
Architectures: amd64
Components: main
Description: ShapeIt APT-Repository
Date: $(date -Ru)
MD5Sum:
 $PKGS_MD5 $PKGS_SIZE main/binary-amd64/Packages
 $PKGS_GZ_MD5 $PKGS_GZ_SIZE main/binary-amd64/Packages.gz
SHA256:
 $PKGS_SHA256 $PKGS_SIZE main/binary-amd64/Packages
 $PKGS_GZ_SHA256 $PKGS_GZ_SIZE main/binary-amd64/Packages.gz
EOF

# ── Schritt 5: GPG-Signierung ─────────────────────────────────────────────────
echo "[ 5/5 ] GPG-Signierung..."

GPG_KEY=$(gpg --list-secret-keys --with-colons "$GPG_KEY_ID" 2>/dev/null | grep '^sec' | head -1 | cut -d: -f5)

if [[ -n "$GPG_KEY" ]]; then
    # InRelease: Release + Signatur in einer Datei (moderner Standard)
    gpg --batch --yes \
        --local-user "$GPG_KEY_ID" \
        --clearsign \
        --output "$REPO_DIR/dists/stable/InRelease" \
        "$RELEASE_FILE"

    # Release.gpg: separate Signatur (Kompatibilität mit älteren apt-Versionen)
    gpg --batch --yes \
        --local-user "$GPG_KEY_ID" \
        --armor --detach-sign \
        --output "$REPO_DIR/dists/stable/Release.gpg" \
        "$RELEASE_FILE"

    echo "        GPG-Signierung erfolgreich (Schlüssel: $GPG_KEY)"
else
    echo "        Kein GPG-Schlüssel '$GPG_KEY_ID' gefunden – ohne Signierung."
    echo "        Führe zuerst 3-setup-gpg.sh aus, um die Signierung einzurichten."
fi

# ── Ergebnis ───────────────────────────────────────────────────────────────────
echo ""
echo "====================================================="
echo "  Fertig! Repository liegt unter:"
echo "  $REPO_DIR"
echo "====================================================="
echo ""
echo "Verzeichnisstruktur:"
find "$REPO_DIR" -type f | sort | sed 's|'"$REPO_DIR"'/||'
echo ""
echo "Upload zum Server (Beispiel mit rsync – Pfad anpassen):"
echo "  rsync -avz --delete $REPO_DIR/ benutzer@deinserver.de:/var/www/html/apt/"
echo ""
echo "Nutzer fügen das Repository einmalig hinzu mit:"
echo "  curl -fsSL https://deinserver.de/apt/shapeit.gpg | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/shapeit.gpg"
echo "  echo 'deb [arch=amd64 signed-by=/etc/apt/trusted.gpg.d/shapeit.gpg] https://deinserver.de/apt stable main' | sudo tee /etc/apt/sources.list.d/shapeit.list"
echo "  sudo apt update"
echo "  sudo apt install shapeit"
echo ""
