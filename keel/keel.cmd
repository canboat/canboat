@echo off
rem keel launcher: run the prebuilt binary, building or downloading it first.
setlocal
set KEEL_DIR=%~dp0
set BIN=%KEEL_DIR%target\release\keel.exe

if exist "%BIN%" goto run

where cargo >nul 2>nul
if %errorlevel%==0 (
    echo keel: building ^(cargo build --release^) ... 1>&2
    pushd "%KEEL_DIR%" && cargo build --release --quiet && popd || exit /b 1
    goto run
)

echo keel: no cargo toolchain; downloading prebuilt keel.exe ... 1>&2
if not exist "%KEEL_DIR%target\release" mkdir "%KEEL_DIR%target\release"
powershell -NoProfile -Command ^
  "Invoke-WebRequest -Uri 'https://github.com/canboat/canboat/releases/latest/download/keel-windows-x86_64.exe' -OutFile '%BIN%'" || exit /b 1

:run
"%BIN%" %*
