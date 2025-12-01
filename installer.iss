; ================================
;   STM32Builder Windows Installer
; ================================

#define AppName "STM32Builder"
#define AppPublisher "Rodrigue de Guerre"
#define AppExeName "STM32Builder.exe"

; --- Retrieve version from EXE or fallback to "1.0.0" ---
#expr Exec("powershell -command \"(Get-Item 'dist\\STM32Builder\\STM32Builder.exe').VersionInfo.ProductVersion\"", 
    AppVersion, 
    "1.0.0")

; If the PS command fails, this fallback ensures installer still builds
#ifndef AppVersion
#define AppVersion "1.0.0"
#endif

[Setup]
AppId={{A1F8A8B0-4B71-4DED-AE18-6B5C295388F2}
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

SetupIconFile="dist\STM32Builder\icons\app.ico"

UninstallDisplayIcon="{app}\{#AppExeName}"

[Files]
Source: "dist\STM32Builder\{#AppExeName}"; DestDir: "{app}"; Flags: ignoreversion
Source: "dist\STM32Builder\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "toolchain\win\*"; DestDir: "{app}\toolchain\win"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExeName}"; IconFilename="{app}\icons\app.ico"
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"

[Run]
Filename: "{app}\{#AppExeName}"; Description: "Launch STM32Builder"; Flags: nowait postinstall skipifsilent
