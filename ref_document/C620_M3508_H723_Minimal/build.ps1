# Compatibility entry point; the Makefile owns all compiler/linker flags.
param([string]$ToolchainBin = "", [string]$MakePath = "")
$ErrorActionPreference = "Stop"
if (-not $MakePath) {
    $makeCommand = Get-Command mingw32-make.exe -ErrorAction SilentlyContinue
    if ($makeCommand) { $MakePath = $makeCommand.Source }
    else { $MakePath = "D:/vscode/Mingw/mingw64/bin/mingw32-make.exe" }
}
if (-not (Test-Path -LiteralPath $MakePath)) {
    throw "Make not found: $MakePath. Pass -MakePath <mingw32-make.exe>."
}
$makeArgs = @("-j4", "all", "DEBUG=1")
if ($ToolchainBin) { $makeArgs += "GCC_PATH=$($ToolchainBin.Replace('\', '/'))" }
Push-Location $PSScriptRoot
try {
    & $MakePath @makeArgs
    if ($LASTEXITCODE -ne 0) { throw "Make build failed: $LASTEXITCODE" }
} finally {
    Pop-Location
}
