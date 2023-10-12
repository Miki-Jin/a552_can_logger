
@echo off

set BITRATE=1000000
set TOOL_DIR=../src/
set MODEL="A552"
set EXEC=

echo CAN Python Log configuration tool Start!!
set /p CONF="Config(conf_can.txt for detail) update? (y/n) > "
if "%CONF%"=="y" (
    CALL Input_conf_can.bat 
)
CALL read_conf_can.bat

echo CAN BITRATE is %BITRATE%!!  
rem echo %DRATE%
set EXEC=can_a552_logger.py

rem if %MODEL% equ 0 (
rem     set EXEC=can_a552_logger.py
rem ) else if %MODEL% equ 1 (
rem     set EXEC=
rem ) else if %MODEL% equ 2 (
rem     set EXEC=
rem )

if "%~1"=="" (
    python %TOOL_DIR%%EXEC% -b %BITRATE% %CSV% %SYNC% %DRATE% %TEMPC% --can_id %NODEID% --m %SAMPLE% -i %INTERFACE% -c %CHANNEL% %SAVECFG% 
) else (
    rem
    @echo off
    FOR %%a IN (%*) do (
        python %TOOL_DIR%%EXEC% -b %BITRATE% %CSV% %SYNC% %DRATE% %TEMPC% --can_id %NODEID% --m %SAMPLE% -i %INTERFACE% -c %CHANNEL% %SAVECFG% 
    )
)
@echo on
echo "Complete !"



