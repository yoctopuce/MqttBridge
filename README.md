Yoctopuce MqttBridge
====================

Yoctopuce *MqttBridge* is a small tool designed to integrate in Home Assistant devices installed on another site without access to the local network, or even in the middle of nowhere with only a GSM connection.

It's designed to be installed on a machine on your local network, typically on the machine running the MQTT *broker* (mosquitto or similar). It's a command line tool and consumes very few resources. It is available for most platforms directly in binary form, but we also provide you with its source code (in pure C).

The principle of this tool is very simple: it connects via the Yoctopuce API to an instance of [VirtualHub (for Web)](/EN/article/new-a-virtualhub-that-works-through-the-web) and connects it to a local MQTT *broker*, exactly as a local YoctoHub or VirtualHub configured to make MQTT-type HTTP callbacks would have done. [VirtualHub (for Web)](/EN/article/new-a-virtualhub-that-works-through-the-web) is our free tool to remotely control Yoctopuce devices. It is available free of charge with source code, and you can install it on the web host of your choice.

The *topics* used by **MqttBridge** are the same as those managed by Yoctopuce hubs, described in detail in [this article](https://www.yoctopuce.com/EN/article/extending-mqtt-support). The informative *MQTT discovery* topics for direct integration in Home Assistant are of course included. As with the configuration on the hubs, you can choose which *root topic* to use to publish values, as well as deciding whether you want read-only access or whether you want to enable commands to be sent to the modules via MQTT (to switch relays, for example).

If you run the utility with the `--help` option, it provides you with all its options:
```sh
Usage: MqttBridge ...
            --registerHub <yoctohub_ip_or_url>
            --mqtt_broker <hostname_or_ip>
            [--mqtt_port <port> | --mqtts_port <port>]
            [--mqtt_user <username> --mqtt_pass <password>]
            [--root_topic <mqtt_root_topic>]
            [--allow_remote_control]
            [--no_instant_sensor_values]
            [--no_retain_flag]
            [--cacert <ca_cert_file>]
            [--ignore_cert]
Special command-line options:
   -c <conffile>: load config from JSON file to avoid long cmdline
      (config file uses key names identical to the options listed above)
   -g <logfile>: log debug infos into a <logfile>
   -i : install as a service (with the arguments specified)
   -u : uninstall as a service
   -d : run as a daemon (when running as a service on Unix)
   -v : show the version of the executable
```   
To avoid a huge command line with passwords, we recommend using a separate configuration file. The configuration file uses the same keywords, but in JSON format:
```json
{
    "mqtt_broker": "192.168.1.83",
    "mqtt_port": 1883,
    "mqtt_user": "secret-username",
    "mqtt_pass": "secret-password",
    "root_topic": "mqttbridge",
    "allow_remote_control": true,
    "no_instant_sensor_values": true,
    "no_retain_flag": true,
    "ignore_cert": true
}
```
The **MqttBridge** tool is based on our v2.0 programming library, and therefore supports TLS-secured connections, both for contacting VirtualHub (for Web) and for connecting to the MQTT server. However, in this case, you'll either need to provide the `cacert` option for certificates of authority to verify the identity of the servers, or set the `ignore_cert` option to disable certificate verification.

## In conclusion
In addition to enabling the integration of remote Yoctopuce modules in Home Assistant, this new tool can naturally also be used with a local YoctoHub on which you have already configured another HTTP callback, for example to store data on an InfluxDB database. You now have the choice between an MQTT connection initiated entirely autonomously by the hub, or initiated by a machine on your local network.

Please note that as this is a brand-new tool, it's certainly not entirely free of bugs. Please feel free to report them to `support@yoctopuce.com`, and we'll do our best to correct them as quickly as possible.
