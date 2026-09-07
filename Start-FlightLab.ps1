$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$flightLabPython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $flightLabPython)) {
    py -3.12 -m venv .venv
    & $flightLabPython -m pip install -e .
}
& $flightLabPython -m fwrl.app
