; ================================
;   STM32Builder Windows Installer
; ================================

#define AppName "STM32Builder"
#define AppVersion "1.0.0"
#define AppPublisher "Rodrigue de Guerre"
#define AppExeName "STM32Builder.exe"

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
SetupIconFile="icons\app.ico"
UninstallDisplayIcon="{app}\{#AppExeName}"

[Files]
; Main executable
Source: "dist\STM32Builder\{#AppExeName}"; DestDir: "{app}"; Flags: ignoreversion

; Include PyInstaller output
Source: "dist\STM32Builder\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

; Include Windows toolchain
Source: "toolchain\win\*"; DestDir: "{app}\toolchain\win"; Flags: ignoreversion recursesubdirs createallsubdirs
; Include the firmware sources
Source: "firmware\*"; DestDir: "{app}\Accoustic-Controller"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExeName}"
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"

[Run]
Filename: "{app}\{#AppExeName}"; Description: "Launch STM32Builder"; Flags: nowait postinstall skipifsilent

[UninstallDelete]
; This handles the deletion of the folder if it's empty, 
; but for folders with user data, the [Code] section below is safer.
Type: filesandordirs; Name: "{userdocs}\STM32Builder"

[Code]
procedure CurUninstallStepChanged(UninstallStep: TUninstallStep);
var
  DocsPath: String;
begin
  if UninstallStep = usPostUninstall then
  begin
    // Locate the Documents\STM32Builder folder
    DocsPath := ExpandConstant('{userdocs}\STM32Builder');
    
    // Check if it exists and ask or just delete
    if DirExists(DocsPath) then
    begin
      if MsgBox('Do you want to delete the user workspace (firmware projects and parameters) in Documents\STM32Builder?', 
        mbConfirmation, MB_YESNO) = IDYES then
      begin
        // Use RemoveDir with 'true' to delete all files and subfolders
        DelTree(DocsPath, True, True, True);
      end;
    end;
  end;
end;