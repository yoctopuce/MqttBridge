@ECHO OFF

:compile
echo.
echo Build MqttBridge
echo ================
cd Binaries\
call make.bat %1
IF ERRORLEVEL 1 goto error
cd ..\
echo done
goto end

:error
echo error
exit /b 1
:end