@echo off

rem setlocal

REM 設定ファイルパス
set CONFFILE=./conf_can.txt

REM 設定ファイルが存在するか確認する
if not exist %CONFFILE% (
    echo ERROR: Not found %CONFFILE%
    exit /b 1
)

REM 設定ファイルを読み込む
for /f "usebackq tokens=1,2 delims==" %%a in ("%CONFFILE%") do (
    set %%a=%%b
)
rem echo %IMU%
rem endlocal
exit /b 0