; =============================================================================
;  ShapeIt.iss  -  Inno Setup Script fuer den Windows-Installer von ShapeIt
; =============================================================================
;
;  Erzeugt eine Setup.exe (shapeit-setup-VERSION.exe) mit:
;    - Setup-Assistent (Deutsch/Englisch)
;    - Installation der self-contained ShapeIt.Avalonia-Anwendung
;    - Startmenue-Eintrag (+ optional Desktop-Verknuepfung)
;    - Deinstallationsprogramm (Uninstaller)
;
;  Dieses Script wird NICHT direkt aufgerufen, sondern ueber
;  scripts\windows-packaging\2-installer.ps1, das die Werte als /D-Defines
;  uebergibt (Version, Quellverzeichnis, Icon, Ausgabeverzeichnis).
;
;  Manueller Aufruf (Beispiel):
;    ISCC.exe /DAppVersion=0.8.3 ShapeIt.iss
; =============================================================================

; ── Vorgaben (werden von 2-installer.ps1 per /D ueberschrieben) ───────────────
#ifndef AppVersion
  #define AppVersion "0.0.0"
#endif
; Quellverzeichnis = self-contained Publish-Ausgabe (aus 1-build.ps1)
#ifndef PublishDir
  #define PublishDir "..\..\..\publish\win-x64"
#endif
; Ausgabeverzeichnis fuer die fertige Setup.exe
#ifndef OutputDir
  #define OutputDir "..\..\..\publish"
#endif
; Icon der Setup.exe und der Verknuepfungen
#ifndef IconFile
  #define IconFile "..\..\..\ShapeIt.Avalonia\Logo.ico"
#endif

#define AppName "ShapeIt"
#define AppPublisher "cadability.de"
#define AppURL "https://www.cadability.de/ShapeIt"
#define AppExe "ShapeIt.exe"

[Setup]
; AppId identifiziert die Anwendung dauerhaft (fuer Updates/Deinstallation).
; NIEMALS aendern - sonst erkennt Windows eine neue Installation nicht als Update.
AppId={{7170736E-7CBF-4129-83CA-30F3E3F606C8}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher={#AppPublisher}
AppPublisherURL={#AppURL}
AppSupportURL={#AppURL}
AppUpdatesURL={#AppURL}
WizardStyle=modern
DefaultDirName={autopf}\{#AppName}
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
UninstallDisplayIcon={app}\{#AppExe}
UninstallDisplayName={#AppName}
OutputDir={#OutputDir}
OutputBaseFilename=shapeit-setup-{#AppVersion}
SetupIconFile={#IconFile}
Compression=lzma2/max
SolidCompression=yes
; Nur 64-bit (die Anwendung wird als win-x64 self-contained gebaut).
; "x64compatible" erlaubt auch ARM64-Geraete per x64-Emulation.
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
; Standard: Installation pro Benutzer ohne Administratorrechte (kein UAC-Dialog).
; Der Anwender kann im Assistenten auch "fuer alle Benutzer" waehlen (dann UAC).
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog

[Languages]
Name: "german";  MessagesFile: "compiler:Languages\German.isl"
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
; Gesamte self-contained Ausgabe ins Installationsverzeichnis kopieren.
Source: "{#PublishDir}\*"; DestDir: "{app}"; Flags: recursesubdirs createallsubdirs ignoreversion

[Icons]
Name: "{group}\{#AppName}";                       Filename: "{app}\{#AppExe}"
Name: "{group}\{cm:UninstallProgram,{#AppName}}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#AppName}";                 Filename: "{app}\{#AppExe}"; Tasks: desktopicon

[Run]
; Nach der Installation optional direkt starten.
Filename: "{app}\{#AppExe}"; Description: "{cm:LaunchProgram,{#AppName}}"; Flags: nowait postinstall skipifsilent
