@echo off
setlocal
rem Run servotools straight from a clone, without installing it.
rem Installed users just run `servo`; see README.md.
set "HERE=%~dp0"
set "PY=%HERE%.venv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"

"%PY%" -c "import serial" >nul 2>nul
if errorlevel 1 (
  echo(
  echo   pyserial is missing. Install servotools with:
  echo(
  echo     pip install -e "%HERE%"
  echo(
  exit /b 1
)

"%PY%" -c "import sys; sys.path.insert(0, r'%HERE%src'); from servotools.cli import main; sys.exit(main())" %*
