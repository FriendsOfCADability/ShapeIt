#!/bin/bash
# =============================================================================
#  3-setup-gpg.sh  –  GPG-Signierungsschlüssel einmalig einrichten
# =============================================================================
#
#  Dieses Skript nur EINMAL ausführen!
#  Es erstellt ein GPG-Schlüsselpaar für die Repository-Signierung und
#  exportiert den öffentlichen Schlüssel, den Nutzer installieren müssen.
#
#  Aufruf:
#    bash 3-setup-gpg.sh
# =============================================================================

set -euo pipefail

KEY_NAME="ShapeIt Repository"
KEY_EMAIL="shapeit@cadability.de"
KEY_OUTPUT="shapeit.gpg"     # Öffentlicher Schlüssel – auf den Server hochladen

echo ""
echo "====================================================="
echo "  GPG-Schlüssel für ShapeIt Repository einrichten"
echo "====================================================="
echo ""

# Prüfen ob der Schlüssel bereits existiert
if gpg --list-secret-keys --with-colons "$KEY_NAME" 2>/dev/null | grep -q '^sec'; then
    echo "Ein GPG-Schlüssel mit dem Namen '$KEY_NAME' existiert bereits."
    echo "Überspringe Schlüsselerzeugung."
else
    echo "Erzeuge GPG-Schlüsselpaar (RSA 4096 Bit)..."
    echo "(Dies kann einige Sekunden dauern)"
    echo ""

    # Schlüssel ohne Passwort erstellen (für automatisiertes Signieren)
    gpg --batch --gen-key << EOF
%no-protection
Key-Type: RSA
Key-Length: 4096
Subkey-Type: RSA
Subkey-Length: 4096
Name-Real: $KEY_NAME
Name-Email: $KEY_EMAIL
Expire-Date: 0
%commit
EOF

    echo ""
    echo "Schlüssel erfolgreich erstellt."
fi

# Öffentlichen Schlüssel exportieren (armored/ASCII = für Browser herunterladbar)
echo ""
echo "Exportiere öffentlichen Schlüssel nach: $KEY_OUTPUT"
gpg --armor --export "$KEY_NAME" > "$KEY_OUTPUT"

# Fingerabdruck anzeigen
echo ""
echo "Schlüssel-Fingerabdruck:"
gpg --fingerprint "$KEY_NAME"

echo ""
echo "====================================================="
echo "  Fertig!"
echo "====================================================="
echo ""
echo "Nächste Schritte:"
echo ""
echo "  1. Öffentlichen Schlüssel auf den Server hochladen:"
echo "     scp $KEY_OUTPUT benutzer@deinserver.de:/var/www/html/apt/"
echo ""
echo "  2. Ab jetzt signiert 2-package.sh automatisch mit diesem Schlüssel."
echo ""
echo "  WICHTIG: Der private Schlüssel bleibt nur auf diesem Rechner!"
echo "  Sichere ihn mit:"
echo "    gpg --armor --export-secret-keys '$KEY_NAME' > shapeit-private-key-BACKUP.asc"
echo "  Bewahre das Backup sicher auf (z.B. verschlüsselter USB-Stick)."
echo ""
