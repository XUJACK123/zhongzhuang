param([string]$Gcc = 'D:/vscode/Mingw/mingw64/bin/gcc.exe')
$ErrorActionPreference='Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    New-Item -ItemType Directory -Force -Path build | Out-Null
    $common=@('-std=c11','-Wall','-Wextra','-Werror','-ICore/Inc','-DDM_USE_MULTI=0')
    $app=@('tests/test_dm8009_app.c','Core/Src/dm8009_app.c','Core/Src/dm8009_step.c')
    & $Gcc @common '-DDM_FIXED_CAN_ID=0' @app '-o' 'build/test_dm8009_app.exe'
    if($LASTEXITCODE){throw 'Auto-ID host compilation failed'}
    & './build/test_dm8009_app.exe'
    if($LASTEXITCODE){throw 'Auto-ID tests failed'}
    & $Gcc @common @app '-o' 'build/test_dm8009_fixed.exe'
    if($LASTEXITCODE){throw 'Fixed-ID host compilation failed'}
    & './build/test_dm8009_fixed.exe'
    if($LASTEXITCODE){throw 'Fixed-ID tests failed'}
    & $Gcc @common '-DDM_FIXED_CAN_ID=0' '-DDM_ID_SCAN_ONLY=1' 'tests/test_dm8009_idscan.c' 'Core/Src/dm8009_app.c' 'Core/Src/dm8009_step.c' '-o' 'build/test_dm8009_idscan.exe'
    if($LASTEXITCODE){throw 'ID-only host compilation failed'}
    & './build/test_dm8009_idscan.exe'
    if($LASTEXITCODE){throw 'ID-only tests failed'}
    & $Gcc @common 'tests/test_dm8009_protocol.c' '-o' 'build/test_dm8009_protocol.exe'
    if($LASTEXITCODE){throw 'Protocol host compilation failed'}
    & './build/test_dm8009_protocol.exe'
    if($LASTEXITCODE){throw 'Protocol tests failed'}
    $multi=@('-std=c11','-Wall','-Wextra','-Werror','-ICore/Inc','-DDM_USE_MULTI=1')
    & $Gcc @multi @app '-o' 'build/test_dm8009_multi.exe'
    if($LASTEXITCODE){throw 'Multi host compilation failed'}
    & './build/test_dm8009_multi.exe'
    if($LASTEXITCODE){throw 'Multi tests failed'}
    & $Gcc @multi 'tests/test_dm8009_direct.c' 'Core/Src/dm8009_direct.c' '-o' 'build/test_dm8009_direct.exe'
    if($LASTEXITCODE){throw 'Direct host compilation failed'}
    & './build/test_dm8009_direct.exe'
    if($LASTEXITCODE){throw 'Direct tests failed'}
    Write-Output 'All DM8009 host checks PASS'
} finally {Pop-Location}
