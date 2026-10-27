# Arranca la distro de WSL al iniciar sesion en Windows, para que station (restart: unless-stopped) y el trabajador de
# render (servicio systemd de usuario con linger) esten arriba sin abrir una terminal. Requiere en %USERPROFILE%\.wslconfig
# [general] instanceIdleTimeout=-1 y [wsl2] vmIdleTimeout=-1: sin eso WSL apaga la distro a los ~15 s.
# Uso (PowerShell del usuario, no hace falta administrador):
#   powershell -ExecutionPolicy Bypass -File .\arranque_wsl.ps1 [-Distro Ubuntu-24.04] [-Quitar]
param([string]$Distro = 'Ubuntu-24.04', [switch]$Quitar)
$ErrorActionPreference = 'Stop'
$nombre = 'ATP - arrancar WSL'
if ($Quitar) {
  Unregister-ScheduledTask -TaskName $nombre -Confirm:$false -ErrorAction SilentlyContinue
  Write-Host "Tarea '$nombre' quitada."
  return
}
# `--exec /bin/true` despierta la distro (systemd arranca docker, station y atp-render) y sale enseguida
$accion = New-ScheduledTaskAction -Execute "$env:WINDIR\System32\wsl.exe" -Argument "-d $Distro --exec /bin/true"
$disparo = New-ScheduledTaskTrigger -AtLogOn -User "$env:USERDOMAIN\$env:USERNAME"
$disparo.Delay = 'PT30S'   # deja terminar el arranque de la sesion
$ajustes = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -ExecutionTimeLimit (New-TimeSpan -Minutes 5)
$quien = New-ScheduledTaskPrincipal -UserId "$env:USERDOMAIN\$env:USERNAME" -LogonType Interactive -RunLevel Limited
Register-ScheduledTask -TaskName $nombre -Action $accion -Trigger $disparo -Settings $ajustes -Principal $quien `
  -Description 'Despierta la distro de WSL al iniciar sesion (Estacion ATP: station y trabajador de render).' -Force | Out-Null
Write-Host "Tarea '$nombre' registrada: al iniciar sesion ejecuta wsl.exe -d $Distro --exec /bin/true"
