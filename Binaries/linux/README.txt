Start MqttBridge with systemd (Ubuntu, debian, Raspbian, ...)
=============================================================
- 1: copy MqttBridge binary to /usr/sbin
- 2: ensure that the /usr/sbin/MqttBridge is executable with :
    # chmod +x /usr/sbin/MqttBridge
- 3: copy the systemd startup script /etc/systemd/system/
    # cp startup_script/mqttbridge.service /etc/systemd/system/
- 4: ensure that the /etc/systemd/system/mqttbridge.service is executable with :
    # chmod +x /etc/systemd/system/mqttbridge.service
- 5: reload your systemd configuration with
    # systemctl daemon-reload
- 6: ensure that you can start your script with
    # systemctl start mqttbridge.service
- 7: set this service to be started at boot time with
    # systemctl enable mqttbridge.service


Start MqttBridge with System V (old Linux system)
=================================================
- 1: copy the MqttBridge binary to the directory /usr/sbin/
- 2: ensure that the /usr/sbin/MqttBridge is executable with :
    # chmod +x /usr/sbin/MqttBridge
- 3: copy the the file startup_script/MqttBridge to /etc/init.d/
    # cp startup_script/MqttBridge /etc/init.d/
- 4: ensure that the /etc/init.d/MqttBridge is executable with :
    # chmod +x /etc/init.d/MqttBridge
- 5: set this service to be started at boot time with
    # update-rc.d MqttBridge defaults
