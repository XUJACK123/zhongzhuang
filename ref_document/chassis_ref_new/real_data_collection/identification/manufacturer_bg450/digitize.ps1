Add-Type -AssemblyName System.Drawing
$img = New-Object System.Drawing.Bitmap((Join-Path $PSScriptRoot 'product_figure_2.jpg'))
Write-Output ('IMAGE_DIMS ' + $img.Width + 'x' + $img.Height)
$result = @()
foreach ($value in @(10,20,30,40,50,60,70,80,85)) {
 $x = [int][Math]::Round(492 + $value * 4.05)
 $ys = @()
 for ($y=170;$y -le 315;$y++) {
  $orange = $false
  for ($col=$x-2;$col -le $x+2;$col++) {
   $pixel = $img.GetPixel($col,$y)
   if ($pixel.R -gt 220 -and ($pixel.R-$pixel.G) -gt 5 -and ($pixel.G-$pixel.B) -gt 5) {$orange = $true}
  }
  if ($orange) {$ys += $y}
 }
 $groups = @();$group = @();$prev = -100
 foreach ($y in $ys) { if ($y-$prev -gt 2 -and $group.Count -gt 0) { $groups += ,$group;$group=@() };$group+=$y;$prev=$y }
 if ($group.Count -gt 0) {$groups+=,$group}
 $centres = @($groups | ForEach-Object {($_ | Measure-Object -Average).Average})
 if ($centres.Count -ne 3) {throw "Expected three curves at x=$value, got $($centres.Count), rows=$ys"}
 $forces = @($centres | ForEach-Object {[Math]::Round(100+(331-$_)*700/179,1)})
 $result += [PSCustomObject]@{x_axis_value=$value;pixel_x=$x;pixel_y=$centres;force_15MPa_n=$forces[0];force_12MPa_n=$forces[1];force_9MPa_n=$forces[2]}
}
$img.Dispose()
$json = $result | ConvertTo-Json -Depth 6
[System.IO.File]::WriteAllText((Join-Path $PSScriptRoot 'digitized_catalogue_axis_values.json'),$json,[System.Text.UTF8Encoding]::new($false))
Write-Output $json
