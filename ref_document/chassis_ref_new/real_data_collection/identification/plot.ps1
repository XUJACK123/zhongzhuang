$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Windows.Forms.DataVisualization
Add-Type -AssemblyName System.Drawing
$plotData = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'plot_data.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$chart = New-Object System.Windows.Forms.DataVisualization.Charting.Chart
$chart.Width = 1400
$chart.Height = 1000
$chart.BackColor = [System.Drawing.Color]::White
$mainTitle = $chart.Titles.Add('Command / response on the supported fixture - protocol feedback, not independent torque calibration')
$mainTitle.Font = New-Object System.Drawing.Font('Arial',14)
$colors = @('#2563eb','#f59e0b','#16a34a')
$panel = 0
foreach ($item in $plotData) {
    foreach ($kind in @('torque','angle')) {
        $name = 'area' + $panel
        $area = New-Object System.Windows.Forms.DataVisualization.Charting.ChartArea($name)
        $left = if ($kind -eq 'torque') { 0 } else { 50 }
        $top = if ($item.selected -eq 1) { 7 } else { 53 }
        $area.Position = New-Object System.Windows.Forms.DataVisualization.Charting.ElementPosition($left,$top,48,45)
        $side = if ($item.selected -eq 1) { 'Right DM3' } else { 'Left DM2' }
        if ($kind -eq 'torque') {
            $area.AxisX.Title = 'Nominal requested motor torque (Nm)'
            $area.AxisY.Title = $side + ' - DM reported torque (Nm)'
            $area.AxisY.IsStartedFromZero = $true
        } else {
            $area.AxisX.Title = 'Requested current (A) | blue: shaft, orange: derived knee'
            $area.AxisY.Title = $side + ' - change from initial angle (deg)'
            $area.AxisY.IsStartedFromZero = $false
        }
        foreach ($axis in @($area.AxisX,$area.AxisY)) {
            $axis.TitleFont = New-Object System.Drawing.Font('Arial',10)
            $axis.LabelStyle.Font = New-Object System.Drawing.Font('Arial',9)
            $axis.LabelStyle.Format = '0.0'
            $axis.MajorGrid.LineColor = [System.Drawing.ColorTranslator]::FromHtml('#e5e7eb')
        }
        $area.AxisX.Minimum = 0
        $chart.ChartAreas.Add($area)
        $n = 0
        foreach ($line in $item.$kind) {
            $series = New-Object System.Windows.Forms.DataVisualization.Charting.Series($name + '_' + $n)
            $series.ChartArea = $name
            $series.ChartType = [System.Windows.Forms.DataVisualization.Charting.SeriesChartType]::Point
            $series.MarkerStyle = [System.Windows.Forms.DataVisualization.Charting.MarkerStyle]::Circle
            $series.MarkerSize = 4
            $series.Color = [System.Drawing.ColorTranslator]::FromHtml($colors[$n % $colors.Count])
            $series.IsVisibleInLegend = $false
            foreach ($point in $line.points) { [void]$series.Points.AddXY([double]$point[0],[double]$point[1]) }
            $chart.Series.Add($series)
            $n++
        }
        $panel++
    }
}
$outPath = Join-Path $PSScriptRoot 'motor_response.png'
$chart.SaveImage($outPath,[System.Windows.Forms.DataVisualization.Charting.ChartImageFormat]::Png)
$chart.Dispose()
Write-Output $outPath
