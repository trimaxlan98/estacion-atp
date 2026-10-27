# App de escritorio (Windows)

Lanzador nativo de la Estación ATP · Estudio: un `.exe` con icono propio que levanta la estación y el trabajador de render en WSL,
espera a que respondan y abre la app en su propia ventana (Edge en modo aplicación). Al cerrarla pregunta si detiene todo.

    powershell -ExecutionPolicy Bypass -File desktop\build.ps1 -Instalar   # compila y crea los accesos (Escritorio e Inicio)
    desktop\build.ps1 -Frontend                                            # además reconstruye antes el frontend en WSL
    dist\EstacionATP\EstacionATP.exe --detener                             # solo detiene la estación y el trabajador

Compila con el `csc.exe` de .NET Framework que trae Windows (sin instalar nada). `dist/` no se versiona. Requiere el clon
`~/estacion-atp` en la distro `Ubuntu-24.04` .
