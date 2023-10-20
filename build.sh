#!/bin/bash
cd Binaries
echo "Build MqttBridge"
echo "================"
make $1 || exit 1
cd ../
echo -e "done"
