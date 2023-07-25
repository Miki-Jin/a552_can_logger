
@echo off

set BITRATE=1000000
set TOOL_DIR=../tools_python/
set MODEL="A552"
set BIT16=
set EXEC=can_g552pc1_logger.py

set /p  COMX=<MIU_COM.txt
rem set /p  BAURATE=<BAURATE.txt
rem CALL read_baurate.txt
echo CAN Python Logger Start!!


set /p CONF="Config(conf_can.txt for detail) update? (y/n) > "
if "%CONF%"=="y" (
    CALL Input_conf_can.bat 
)
CALL read_conf_can.bat

echo CAN BITRATE is %BITRATE%!!  
rem echo %DRATE%

if %MODEL% equ 0 (
    set EXEC=can_a552_logger_std.py
) else if %MODEL% equ 1 (
    set EXEC=can_g552pc1_logger.py
) else if %MODEL% equ 2 (
    set EXEC=can_g552pj1_logger.py
)

rem set EXEC=can_a552_logger.py

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



