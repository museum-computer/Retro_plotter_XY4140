# Vygeneruje 10 testovacich HPGL souboru pro XY 4140 do slozky sd_card.
#   powershell -ExecutionPolicy Bypass -File tools\make_samples.ps1
#   powershell -File tools\make_samples.ps1 -OutDir D:\obsah_karty
#
# Souradnice ve skriptu jsou v milimetrech plochy 260 x 185 mm,
# na vystupu se prevadeji na HPGL jednotky 0.025 mm.

param(
  [string]$OutDir = (Join-Path $PSScriptRoot '..\sd_card')
)

if ([string]::IsNullOrEmpty($OutDir)) { $OutDir = (Join-Path $PSScriptRoot '..\sd_card') }

$ErrorActionPreference = 'Stop'
$W = 10400
$H = 7400
$MMW = 260.0
$MMH = 185.0
$Script:sb = $null
$Script:penDown = $false
$Script:hasPos = $false
$Script:forceLift = $false
$Script:lastX = 0
$Script:lastY = 0

function Reset-Plot {
  $Script:sb = New-Object System.Text.StringBuilder
  $Script:penDown = $false
  $Script:hasPos = $false
  $Script:forceLift = $false
  [void]$Script:sb.AppendLine('IN;PA;SP1;')
}

# Pred dalsi useckou zvednout pero, aby nevznikla spojovaci cara mezi
# dvema nesouvisejicimi kresbami.
function BeginStroke { $Script:forceLift = $true }

# pri penez dolu kresli, pri penez nahoru jen sune pero.
# zapisuje pouze potrebny pohyb, a pri zacatku tahu nejdriv sune pero nahoru
# na zacatek usecky. Bez toho by PD bez predchoziho PU vytvoril linku
# z vychodni polohy.
function Seg {
  param([bool]$Pen, [double]$X1, [double]$Y1, [double]$X2, [double]$Y2)
  if ($Script:forceLift) { $Script:penDown = $false; $Script:forceLift = $false }
  $a = [int][math]::Round($X1 / $MMW * $W)
  $b = [int][math]::Round($Y1 / $MMH * $H)
  $c = [int][math]::Round($X2 / $MMW * $W)
  $d = [int][math]::Round($Y2 / $MMH * $H)
  $cmd = 'PU'
  if ($Pen) { $cmd = 'PD' }
  $at = $Script:hasPos -and ($Script:lastX -eq $a) -and ($Script:lastY -eq $b)
  if ($at -and ($Script:penDown -eq $Pen)) {
    [void]$Script:sb.AppendLine("$cmd$c,$d;")
  } else {
    if ($Pen -and $Script:penDown) { [void]$Script:sb.AppendLine("PD$a,$b;") }
    else { [void]$Script:sb.AppendLine("PU$a,$b;") }
    [void]$Script:sb.AppendLine("$cmd$c,$d;")
  }
  $Script:penDown = $Pen
  $Script:hasPos = $true
  $Script:lastX = $c
  $Script:lastY = $d
}

function Rect {
  param([bool]$Pen, [double]$X1, [double]$Y1, [double]$X2, [double]$Y2)
  Seg $Pen $X1 $Y1 $X2 $Y1
  Seg $Pen $X2 $Y1 $X2 $Y2
  Seg $Pen $X2 $Y2 $X1 $Y2
  Seg $Pen $X1 $Y2 $X1 $Y1
}

function Poly {
  param([bool]$Pen, [object[]]$Pts)
  $prevX = $null
  $prevY = $null
  foreach ($p in $Pts) {
    if ($null -ne $prevX) { Seg $Pen $prevX $prevY $p[0] $p[1] }
    $prevX = $p[0]
    $prevY = $p[1]
  }
}

function Arc {
  param([double]$CX, [double]$CY, [double]$R, [double]$A1, [double]$A2, [int]$Step = 3)
  $prevX = $null
  $prevY = $null
  for ($a = $A1; $a -le ($A2 + 0.001); $a += $Step) {
    $rad = $a * [math]::PI / 180.0
    $x = $CX + $R * [math]::Cos($rad)
    $y = $CY + $R * [math]::Sin($rad)
    if ($null -ne $prevX) { Seg $true $prevX $prevY $x $y }
    $prevX = $x
    $prevY = $y
  }
}

function Star {
  param([double]$CX, [double]$CY, [double]$Rout, [double]$Rin, [int]$N)
  $pts = @()
  for ($i = 0; $i -lt (2 * $N); $i++) {
    $rad = (90.0 + 180.0 * $i / $N) * [math]::PI / 180.0
    $rr = $Rout
    if (($i % 2) -eq 1) { $rr = $Rin }
    $pts += , @(($CX + $rr * [math]::Cos($rad)), ($CY + $rr * [math]::Sin($rad)))
  }
  $pts += , @($pts[0][0], $pts[0][1])
  Poly $true $pts
}

function Save-Plot {
  param([string]$Name)
  [void]$Script:sb.AppendLine('PU;SP0;')
  $text = $Script:sb.ToString()
  $path = Join-Path $OutDir $Name
  [System.IO.File]::WriteAllText($path, $text, [System.Text.Encoding]::ASCII)
  $info = New-Object System.IO.FileInfo($path)
  Write-Host ("{0,-18} {1,7} B" -f $Name, $info.Length)
}

if (-not (Test-Path -LiteralPath $OutDir)) {
  New-Item -ItemType Directory -Path $OutDir | Out-Null
}

# 01 ramec plochy a orezy v rohich
Reset-Plot
BeginStroke; Rect $true 6 6 254 179
BeginStroke; Rect $true 16 16 244 169
foreach ($p in @(@(6,6),@(254,6),@(254,179),@(6,179))) {
  BeginStroke
  Seg $true ($p[0] - 4) $p[1] ($p[0] + 4) $p[1]
  Seg $true $p[0] ($p[1] - 4) $p[0] ($p[1] + 4)
}
BeginStroke; Seg $true 130 8 130 177
BeginStroke; Seg $true 8 92.5 252 92.5
Save-Plot '01_frame.plt'

# 02 mrizka
Reset-Plot
for ($i = 0; $i -le 20; $i++) {
  $x = 3 + $i * 254 / 20
  BeginStroke; Seg $true $x 3 $x 182
}
for ($i = 0; $i -le 13; $i++) {
  $y = 3 + $i * 179 / 13
  BeginStroke; Seg $true 3 $y 257 $y
}
Save-Plot '02_grid.plt'

# 03 pravitka po spodni a leve strane
Reset-Plot
BeginStroke; Seg $true 3 6 257 6
for ($i = 0; $i -le 254; $i++) {
  $x = 3 + $i
  $len = 2
  if (($i % 10) -eq 0) { $len = 6 }
  if (($i % 50) -eq 0) { $len = 9 }
  BeginStroke; Seg $true $x 6 $x (6 + $len)
}
BeginStroke; Seg $true 6 3 6 182
for ($i = 0; $i -le 179; $i++) {
  $y = 3 + $i
  $len = 2
  if (($i % 10) -eq 0) { $len = 6 }
  if (($i % 50) -eq 0) { $len = 9 }
  BeginStroke; Seg $true 6 $y (6 + $len) $y
}
Save-Plot '03_ruler.plt'

# 04 soustredne kruznice
Reset-Plot
for ($r = 10; $r -le 90; $r += 10) { BeginStroke; Arc 130 92.5 $r 0 360 3 }
BeginStroke; Arc 130 92.5 45 0 360 1
Save-Plot '04_circle.plt'

# 05 Archimedova spirala
Reset-Plot
$pts = @()
for ($a = 0; $a -le 720; $a += 2) {
  $rad = $a * [math]::PI / 180.0
  $r = 2.0 + 0.115 * $a
  $pts += , @((130 + $r * [math]::Cos($rad)), (92.5 + $r * [math]::Sin($rad)))
}
Poly $true $pts
Save-Plot '05_spiral.plt'

# 06 Lissajousuv obrazec
Reset-Plot
$pts = @()
for ($a = 0; $a -le 360; $a += 2) {
  $rad = $a * [math]::PI / 180.0
  $pts += , @((130 + 115 * [math]::Sin(3 * $rad)), (92.5 + 80 * [math]::Sin(2 * $rad)))
}
Poly $true $pts
Save-Plot '06_lissajous.plt'

# 07 petihviezda a sedmihviezda
Reset-Plot
BeginStroke; Star 90 92.5 70 28 5
BeginStroke; Star 190 92.5 45 20 7
Save-Plot '07_star.plt'

# 08 kresba pod uhlem 45 deg v kruhu
# POZOR: v PowerShellu se nerozlisuje velikost pismen, $h by prepisalo $H (vysku plochy)
Reset-Plot
BeginStroke; Arc 130 92.5 62 0 360 3
$rad2 = 62.0 * 62.0
for ($c = -230; $c -le 230; $c += 6) {
  $q = $rad2 / 2 - $c * $c / 4
  if ($q -le 0) { continue }
  $half = [math]::Sqrt($q)
  BeginStroke
  Seg $true (130 - $c / 2 - $half) (92.5 + $c / 2 + $half) (130 - $c / 2 + $half) (92.5 + $c / 2 - $half)
}
Save-Plot '08_hatch.plt'

# 09 tri sinusovky
Reset-Plot
foreach ($cfg in @(@(1, 18.0), @(2, 38.0), @(3, 58.0))) {
  $f = $cfg[0]
  $amp = $cfg[1]
  $prevX = $null
  $prevY = $null
  BeginStroke
  for ($x = 4; $x -le 256; $x += 1) {
    $y = 92.5 + $amp * [math]::Sin(2 * [math]::PI * $f * $x / 253.0)
    if ($null -ne $prevX) { Seg $true $prevX $prevY $x $y }
    $prevX = $x
    $prevY = $y
  }
}
Save-Plot '09_sine.plt'

# 10 hadavy vypadek po cele plose
Reset-Plot
$rows = 20
for ($i = 0; $i -le $rows; $i++) {
  $y = 4 + $i * (177.0 / $rows)
  BeginStroke
  if (($i % 2) -eq 0) { Seg $true 4 $y 256 $y }
  else { Seg $true 256 $y 4 $y }
}
BeginStroke; Seg $true 4 4 256 181
Save-Plot '10_sweep.plt'

Write-Host ''
Write-Host ('Hotovo, {0} souboru ve slozce {1}' -f (Get-ChildItem -LiteralPath $OutDir -Filter '*.plt').Count, (Resolve-Path -LiteralPath $OutDir).Path)
