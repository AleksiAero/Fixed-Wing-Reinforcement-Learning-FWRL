$ErrorActionPreference = 'Stop'
Start-Process -FilePath 'wsl.exe' -WorkingDirectory $PSScriptRoot -WindowStyle Hidden -ArgumentList @('-d', 'Ubuntu-24.04', '--', 'bash', 'scripts/wsl_python.sh', '-m', 'fwrl.session')
