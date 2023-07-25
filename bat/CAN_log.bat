
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

if %MODEL% equ 0 (
    set EXEC=can_a552_logger.py
) else if %MODEL% equ 1 (
    set EXEC=
) else if %MODEL% equ 2 (
    set EXEC=
)

if "%~1"=="" (
    python %TOOL_DIR%%EXEC% -b %BITRATE% %CSV% %SYNC% %DRATE% --can_id %NODEID% --m %SAMPLE% -i %INTERFACE% -c %CHANNEL% %SAVECFG% 
) else (
    rem
    @echo off
    FOR %%a IN (%*) do (
        python %TOOL_DIR%%EXEC% -b %BITRATE% %CSV% %SYNC% %DRATE% --can_id %NODEID% --m %SAMPLE% -i %INTERFACE% -c %CHANNEL% %SAVECFG% 
    )
)
@echo on
echo "Complete !"



