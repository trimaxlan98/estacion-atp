# Dibuja el icono de la Estación ATP (el mismo motivo que el favicon de la app: antena, haz y satélite) y lo guarda como .ico
# con varios tamaños. Uso: powershell -File hacer_icono.ps1 <salida.ico>
param([Parameter(Mandatory)][string]$Salida)
Add-Type -AssemblyName System.Drawing
function Color([string]$h) { [System.Drawing.ColorTranslator]::FromHtml($h) }
function Dibuja([int]$n) {
    $bmp = New-Object System.Drawing.Bitmap $n, $n
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.SmoothingMode = 'AntiAlias'; $g.Clear([System.Drawing.Color]::Transparent)
    $s = $n / 32.0
    $g.FillEllipse((New-Object System.Drawing.SolidBrush (Color '#070b12')), [single](1*$s), [single](1*$s), [single](30*$s), [single](30*$s))
    $g.DrawEllipse((New-Object System.Drawing.Pen -ArgumentList (Color '#31414f'), ([single](1.5*$s))), [single](2*$s), [single](2*$s), [single](28*$s), [single](28*$s))
    $pen = New-Object System.Drawing.Pen (Color '#22d3ee'), ([single](2.6*$s))
    $pen.StartCap = 'Round'; $pen.EndCap = 'Round'
    $g.DrawBezier($pen, [single](6*$s), [single](22*$s), [single](12.667*$s), [single](10*$s), [single](19.333*$s), [single](10*$s), [single](26*$s), [single](22*$s))
    $g.FillEllipse((New-Object System.Drawing.SolidBrush (Color '#f59e0b')), [single](21.2*$s), [single](5.2*$s), [single](5.6*$s), [single](5.6*$s))
    $g.Dispose()
    $ms = New-Object System.IO.MemoryStream
    $bmp.Save($ms, [System.Drawing.Imaging.ImageFormat]::Png); $bmp.Dispose()
    , $ms.ToArray()
}
$tam = 16, 24, 32, 48, 64, 128, 256
$pngs = @(); foreach ($n in $tam) { $pngs += , (Dibuja $n) }
$fs = [System.IO.File]::Create($Salida); $w = New-Object System.IO.BinaryWriter $fs
$w.Write([uint16]0); $w.Write([uint16]1); $w.Write([uint16]$tam.Count)
$off = 6 + 16 * $tam.Count
for ($i = 0; $i -lt $tam.Count; $i++) {
    $n = $tam[$i]; $b = if ($n -ge 256) { 0 } else { $n }
    $w.Write([byte]$b); $w.Write([byte]$b); $w.Write([byte]0); $w.Write([byte]0)
    $w.Write([uint16]1); $w.Write([uint16]32); $w.Write([uint32]$pngs[$i].Length); $w.Write([uint32]$off)
    $off += $pngs[$i].Length
}
foreach ($p in $pngs) { $w.Write([byte[]]$p) }
$w.Close(); $fs.Close()
"icono: $Salida ($($tam -join ', ') px)"
