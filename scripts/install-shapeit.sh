#!/bin/bash
# =============================================================================
#  install-shapeit.sh  –  ShapeIt installieren und APT-Repository einrichten
# =============================================================================
#
#  Dieses Skript führt der ANWENDER einmalig auf seinem Linux-Rechner aus.
#  Danach installiert und aktualisiert der normale System-Update-Manager ShapeIt.
#
#  Aufruf (als normaler Benutzer, sudo wird intern verwendet):
#    bash install-shapeit.sh
#
#  Oder direkt vom Server:
#    curl -fsSL https://cadability.de/ShapeIt/apt/install-shapeit.sh | bash
# =============================================================================

set -euo pipefail

# ── Konfiguration (hier Server-URL eintragen) ─────────────────────────────────
SERVER_URL="https://cadability.de/ShapeIt/apt"    # ← Diese URL anpassen!
# ──────────────────────────────────────────────────────────────────────────────

GPG_KEYRING="/etc/apt/trusted.gpg.d/shapeit.gpg"
SOURCES_FILE="/etc/apt/sources.list.d/shapeit.list"

echo ""
echo "====================================================="
echo "  ShapeIt installieren"
echo "====================================================="
echo ""

# Prüfen ob apt verfügbar ist (Ubuntu/Debian/Mint)
if ! command -v apt &>/dev/null; then
    echo "Fehler: apt nicht gefunden. Dieses Skript funktioniert nur auf"
    echo "Ubuntu, Debian und Linux Mint."
    exit 1
fi

# Prüfen ob bereits installiert
if [[ -f "$SOURCES_FILE" ]]; then
    echo "Das ShapeIt-Repository ist bereits eingerichtet."
    echo "Starte Update..."
    sudo apt update
    sudo apt upgrade shapeit
    echo ""
    echo "ShapeIt wurde auf die neueste Version aktualisiert."
    exit 0
fi

# Schritt 1: GPG-Schlüssel installieren (damit apt dem Repository vertraut)
echo "[ 1/3 ] GPG-Schlüssel installieren..."
curl -fsSL "$SERVER_URL/shapeit.gpg" \
    | sudo gpg --dearmor -o "$GPG_KEYRING"
echo "        Schlüssel installiert: $GPG_KEYRING"

# Schritt 2: Repository-Quelle hinzufügen
echo "[ 2/3 ] Repository-Quelle hinzufügen..."
echo "deb [arch=amd64 signed-by=$GPG_KEYRING] $SERVER_URL stable main" \
    | sudo tee "$SOURCES_FILE" > /dev/null
echo "        Quelle gespeichert: $SOURCES_FILE"

# Schritt 3: Installieren
echo "[ 3/3 ] ShapeIt installieren..."
sudo apt update -qq
sudo apt install -y shapeit

echo ""
echo "====================================================="
echo "  ShapeIt wurde erfolgreich installiert!"
echo "====================================================="
echo ""
echo "ShapeIt starten:"
echo "  shapeit"
echo "  (oder über das Anwendungsmenü)"
echo ""
echo "Automatische Updates:"
echo "  ShapeIt wird bei der normalen Systemaktualisierung"
echo "  automatisch mit aktualisiert."
echo ""
