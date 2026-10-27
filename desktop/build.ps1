# Compila el lanzador de escritorio de la Estación ATP y crea los accesos directos.
#   powershell -ExecutionPolicy Bypass -File desktop\build.ps1            -> dist\EstacionATP\EstacionATP.exe
#   powershell -ExecutionPolicy Bypass -File desktop\build.ps1 -Instalar  -> además, accesos en el Escritorio y en el menú Inicio
#   -Frontend reconstruye antes el frontend en WSL (lo que sirve la estación).
param([switch]$Instalar, [switch]$Frontend)
$ErrorActionPreference = 'Stop'
$aqui = $PSScriptRoot
$dist = Join-Path (Split-Path $aqui) 'dist\EstacionATP'
New-Item -ItemType Directory -Force $dist | Out-Null
if ($Frontend) {
    wsl.exe -d Ubuntu-24.04 -- bash -lc "cd ~/estacion-atp/station/frontend && nice -n 10 npm run build 2>&1 | tail -1"
}
$ico = Join-Path $dist 'EstacionATP.ico'
& powershell -NoProfile -File (Join-Path $aqui 'hacer_icono.ps1') $ico
$csc = "$env:WINDIR\Microsoft.NET\Framework64\v4.0.30319\csc.exe"
& $csc /nologo /target:winexe /codepage:65001 "/win32icon:$ico" /r:System.Windows.Forms.dll /r:System.Drawing.dll `
    "/out:$dist\EstacionATP.exe" (Join-Path $aqui 'EstacionATP.cs')
if ($LASTEXITCODE -ne 0) { throw 'csc falló' }
"exe: $dist\EstacionATP.exe"
if ($Instalar) {
    $sh = New-Object -ComObject WScript.Shell
    foreach ($dir in [Environment]::GetFolderPath('Desktop'), (Join-Path $env:APPDATA 'Microsoft\Windows\Start Menu\Programs')) {
        $l = $sh.CreateShortcut((Join-Path $dir 'Estación ATP.lnk'))
        $l.TargetPath = "$dist\EstacionATP.exe"; $l.WorkingDirectory = $dist
        $l.IconLocation = "$dist\EstacionATP.exe,0"; $l.Description = 'Estación ATP · Estudio (Co.De Aerospace)'
        $l.Save(); "acceso: $dir\Estación ATP.lnk"
    }
}
