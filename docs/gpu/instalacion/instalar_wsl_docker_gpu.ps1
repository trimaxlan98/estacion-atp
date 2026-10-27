# Ejecutar DESPUES de reiniciar, en PowerShell como administrador. Instala Ubuntu 24.04 en WSL2 y deja Docker + GPU listos.
# Uso:  powershell -ExecutionPolicy Bypass -File .\instalar_wsl_docker_gpu.ps1 [-Usuario local_udi]
# UID 1000: coincide con el usuario `dev` de la imagen (docker-compose.yml, args UID/GID), asi /ws no queda con otro dueno.
# Topes de la VM (para no competir con otros procesos del host): %USERPROFILE%\.wslconfig con [wsl2] processors=4, memory=10GB.
# Servicios permanentes (station, atp-render): ademas [general] instanceIdleTimeout=-1 y [wsl2] vmIdleTimeout=-1, o WSL apaga
# la distro ~15 s despues de cerrar la ultima terminal.
param([string]$Usuario = 'atp')
$ErrorActionPreference = 'Stop'
$distro = 'Ubuntu-24.04'
wsl.exe --set-default-version 2
if (-not ((wsl.exe -l -q) -replace "`0", '' | Select-String -SimpleMatch $distro)) {
  wsl.exe --install -d $distro --no-launch
  & "$env:LOCALAPPDATA\Microsoft\WindowsApps\ubuntu2404.exe" install --root
}
# Usuario sin contrasena interactiva, con sudo sin clave (solo para esta estacion de pruebas)
wsl.exe -d $distro -u root -- bash -lc "id -u $Usuario >/dev/null 2>&1 || useradd -m -u 1000 -s /bin/bash -G sudo $Usuario; echo '$Usuario ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/$Usuario; printf '[user]\ndefault=$Usuario\n[boot]\nsystemd=true\n' > /etc/wsl.conf"
wsl.exe --terminate $distro
# Docker Engine + NVIDIA Container Toolkit dentro de la distribucion
$sh = Join-Path $PSScriptRoot 'docker_gpu_en_wsl.sh'
$wslPath = (wsl.exe -d $distro -- wslpath -a ($sh -replace '\\','/')).Trim()
wsl.exe -d $distro -u root -- bash $wslPath
wsl.exe --terminate $distro
Write-Host 'Listo. Prueba:  wsl -d Ubuntu-24.04 -- docker run --rm --gpus all ubuntu:24.04 nvidia-smi'
