@echo off

setlocal

REM 設定ファイルパス
set CONFFILE=./conf_can.txt

:INPUT_INTERFACE
set INTERFACE=
set /p INTERFACE= "CAN INTERFACE ADAPTER? (pcan/nixnet/) > "
if "%INTERFACE%"=="" goto :INPUT_INTERFACE
echo INTERFACE=%INTERFACE% > conf_can.txt
rem set /p CHANNEL= "CAN CHANNEL No?  (can1 or can2) > "
if "%INTERFACE%"=="pcan" (
    set CHANNEL=PCAN_USBBUS1
) else if "%INTERFACE%"=="nixnet" (
    set /p CHANNEL= "CAN CHANNEL No?  (can1 or can2) > " 
    rem echo CHANNEL=%CHANNEL% >> conf_can.txt
) else (
    set CHANNEL=PCAN_USBBUS2
)
echo CHANNEL=%CHANNEL% >> conf_can.txt
:INPUT_MODEL
set MODEL=
set /p MODEL= "MODEL? (0:A552 1:G552PC1 2:G552PJ) > "
if "%MODEL%"=="" goto :INPUT_MODEL
echo MODEL=%MODEL% >> conf_can.txt

:INPUT_BITRATE
set BITRATE=
set /P BITRATE="CAN bitarate ? > "
if "%BITRATE%"=="" goto :INPUT_BITRATE
echo BITRATE=%BITRATE% >> conf_can.txt

:INPUT_NODEID
set NODEID=
set /P NODEID="Node ID ? > "
if "%NODEID%"=="" goto :INPUT_NODEID
echo NODEID=%NODEID% >> conf_can.txt

:INPUT_SAMPLE
set SAMPLE=
set /P SAMPLE="Samples? > "
if "%SAMPLE%"=="" goto :INPUT_SAMPLE
echo SAMPLE=%SAMPLE% >> conf_can.txt

:INPUT_DRATE
set DRATE=
set /P DRATE="DRATE? (Hz) > "
if "%DRATE%"=="" goto :INPUT_DRATE
echo DRATE=%DRATE% >> conf_can.txt

:INPUT_FILTER
set FILTER=
set /P FILTER="Filter setting(choose from 'K64_FC83', 'K64_FC220', 'K128_FC36', 'K128_FC110', 'K128_FC350', 'K512_FC9','K512_FC16', 'K512_FC60', 'K512_FC210','K512_FC460')? > "
echo FILTER=%FILTER% >> conf_can.txt

:INPUT_SYNC
set SYNC=
set /P SYNC="Sync mode?(y/n) > "
if "%SYNC%"=="y" (
    set SYNC=--sync_hz
) else (
    set SYNC=--drate
)
echo SYNC=%SYNC% >> conf_can.txt

:INPUT_CSV
set CSV=
set /P CSV="csv? (y/n)> "
if "%CSV%"=="" goto :INPUT_CSV
if "%CSV%"=="--outfile" (
    set CSV=--csv
) else (
    set CSV=
)
echo CSV=%CSV% >> conf_can.txt

:INPUT_TEMPC
set TEMPC=
set /P TEMPC="temperature output? (y/n)> "
if "%TEMPC%"=="" goto :INPUT_TEMPC
if "%TEMPC%"=="y" (
    set TEMPC=--tempc
) else (
    set TEMPC=
)
echo TEMPC=%TEMPC% >> conf_can.txt

:INPUT_NOSCALE
set NOSCALE=
set /P NOSCALE="Scaled data output? (y/n)> "
if "%NOSCALE%"=="" goto :INPUT_NOSCALE
if "%NOSCALE%"=="y" (
    set NOSCALE=
) else (
    set NOSCALE=--noscale
)
echo NOSCALE=%NOSCALE% >> conf_can.txt

:INPUT_SAVECFG
set SAVECFG=
set /P SAVECFG="Configuration save ? (y/n)> "
if "%SAVECFG%"=="" goto :INPUT_NOSCALE
if "%SAVECFG%"=="y" (
    set SAVECFG=--svcfg
) else (
    set SAVECFG=
)
echo SAVECFG=%SAVECFG% >> conf_can.txt

rem :INPUT_BIT16
rem set BIT16=
rem set /P BIT16="16bit data output? (y/n)> "
rem if "%BIT16%"=="y" (
rem     set BIT16="--bit16"
rem ) else (
rem    set BIT16=
rem )
rem echo BIT16=%BIT16% >> conf_can.txt

endlocal
exit /b 0