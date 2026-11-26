$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (!(Test-Path $vswhere)) { throw 'Visual Studio C++ Build Tools are required.' }
$vs = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (!$vs) { throw 'Visual Studio C++ Build Tools were not found.' }
$build = Join-Path $root '.pio\build\native-tests'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$sources = @(
  'test\test_wa8ded.cpp', 'src\tnc\tnc.cpp', 'src\tnc\tnc_ded.cpp',
  'src\tnc\mailbox.cpp', 'src\aprs\aprs.cpp', 'src\util\crc.cpp',
  'src\ax25\ax25_addr.cpp', 'src\ax25\ax25_frame.cpp', 'src\ax25\ax25_fcs.cpp',
  'src\ax25\ax25_kiss.cpp', 'src\ax25\ax25_l2.cpp'
) | ForEach-Object { '"' + (Join-Path $root $_) + '"' }
$includes = @('test\native', 'src', 'include', 'variants\devkitv1_e22') |
  ForEach-Object { '/I"' + (Join-Path $root $_) + '"' }
$setup = Join-Path $vs 'VC\Auxiliary\Build\vcvars64.bat'
$command = 'call "' + $setup + '" >nul && cd /d "' + $build +
  '" && cl /nologo /std:c++17 /EHsc /W3 /D_CRT_SECURE_NO_WARNINGS /DAXLORA_VARIANT_DEVKITV1_E22 ' +
  ($includes -join ' ') + ' ' + ($sources -join ' ') +
  ' /Fe:wa8ded-tests.exe && wa8ded-tests.exe'
& $env:ComSpec /c $command
if ($LASTEXITCODE -ne 0) { throw "Native tests failed: $LASTEXITCODE" }
