@echo off
rem keel launcher: bootstrap the venv on first use, then run the real keel.
setlocal
set KEEL_DIR=%~dp0
set VENV=%KEEL_DIR%.venv

if not exist "%VENV%\Scripts\keel.exe" (
    echo keel: setting up virtualenv in %VENV% ... 1>&2
    python -m venv "%VENV%" || exit /b 1
    "%VENV%\Scripts\python" -m pip install --quiet --editable "%KEEL_DIR%." || exit /b 1
)

"%VENV%\Scripts\keel" %*
