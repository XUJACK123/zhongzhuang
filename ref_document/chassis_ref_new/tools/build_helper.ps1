param(
    [Parameter(Mandatory=$true)][ValidateSet('mkdir','stamp','clean')][string]$Operation,
    [Parameter(Mandatory=$true)][string]$Path,
    [string]$Value = ''
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$allowedRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot 'build')).TrimEnd('\')
$targetPath = [IO.Path]::GetFullPath((Join-Path $projectRoot $Path)).TrimEnd('\')
if ($targetPath -ne $allowedRoot -and -not $targetPath.StartsWith($allowedRoot + '\', [StringComparison]::OrdinalIgnoreCase)) {
    throw "Build operation outside project build directory: $targetPath"
}
switch ($Operation) {
    'mkdir' { [IO.Directory]::CreateDirectory($targetPath) | Out-Null }
    'stamp' {
        [IO.Directory]::CreateDirectory((Split-Path -Parent $targetPath)) | Out-Null
        if (-not [IO.File]::Exists($targetPath) -or [IO.File]::ReadAllText($targetPath) -ne $Value) {
            [IO.File]::WriteAllText($targetPath, $Value, (New-Object Text.UTF8Encoding($false)))
        }
    }
    'clean' {
        if (Test-Path -LiteralPath $targetPath) { Remove-Item -LiteralPath $targetPath -Recurse -Force }
    }
}
