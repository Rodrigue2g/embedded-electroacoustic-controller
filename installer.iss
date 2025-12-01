; ================================
;   STM32Builder Windows Installer
; ================================

#define AppName "STM32Builder"
#define AppVersion GetFileVersion("dist\STM32Builder.exe")
#define AppPublisher "Rodrigue de Guerre"
#define AppExeName "STM32Builder.exe"

[Setup]
AppId={A1F8A8B0-4B71-4DED-AE18-6B5C295388F2}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher={#AppPublisher}
DefaultDirName={pf}\{#AppName}
DefaultGroupName={#AppName}
DisableProgramGroupPage=no
OutputDir=installer
OutputBaseFilename=STM32BuilderInstaller
Compression=lzma
SolidCompression=yes
ArchitecturesAllowed=x64
ArchitecturesInstallIn64BitMode=x64
WizardStyle=modern
SetupIconFile="icons\app.ico"
UninstallDisplayIcon="{app}\{#AppExeName}"

[Files]
; Main executable
Source: "dist\STM32Builder\{#AppExeName}"; DestDir: "{app}"; Flags: ignoreversion

; Include PyInstaller bundle files
Source: "dist\STM32Builder\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

; If you bundle toolchain for Windows:
Source: "toolchain\win\*"; DestDir: "{app}\toolchain\win"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExeName}"
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"

[Run]
Filename: "{app}\{#AppExeName}"; Description: "Launch STM32Builder"; Flags: nowait postinstall skipifsilent
