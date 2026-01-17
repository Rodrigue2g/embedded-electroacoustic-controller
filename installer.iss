; ================================
;   EARBuilder Windows Installer
; ================================

#define AppName "EARBuilder"
#define AppVersion "1.0.0"
#define AppPublisher "Rodrigue de Guerre"
#define AppExeName "EARBuilder.exe"

[Setup]
AppId={{A1F8A8B0-4B71-4DED-AE18-6B5C295388F2}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher={#AppPublisher}
DefaultDirName={pf}\{#AppName}
DefaultGroupName={#AppName}
DisableProgramGroupPage=no
OutputDir=installer
OutputBaseFilename=EARBuilderInstaller
Compression=lzma
SolidCompression=yes
ArchitecturesAllowed=x64
ArchitecturesInstallIn64BitMode=x64
WizardStyle=modern
SetupIconFile="icons\app.ico"
UninstallDisplayIcon="{app}\{#AppExeName}"

[Files]
; Main executable
Source: "dist\EARBuilder\{#AppExeName}"; DestDir: "{app}"; Flags: ignoreversion

; Include PyInstaller output
Source: "dist\EARBuilder\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

; Include Windows toolchain
Source: "toolchain\win\*"; DestDir: "{app}\toolchain\win"; Flags: ignoreversion recursesubdirs createallsubdirs
; Include the firmware sources
Source: "firmware\*"; DestDir: "{app}\Accoustic-Controller"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExeName}"
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"

[Run]
Filename: "{app}\{#AppExeName}"; Description: "Launch EARBuilder"; Flags: nowait postinstall skipifsilent