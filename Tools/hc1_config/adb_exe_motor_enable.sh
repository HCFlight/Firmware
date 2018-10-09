#!/bin/bash

adbState=`adb get-state`
device="device"
path=`dirname $0`

if [ $adbState == $device ];then

adb wait-for-device 
adb push  $path/motor_enable.sh /
adb shell chmod +x /motor_enable.sh
adb shell /motor_enable.sh
echo -e "\033[32mHC1 motor enable success!\033[0m"

else

echo -e "\033[31mHC1 motor enable failed!"
echo "HC1 device disconnected"
echo -e "\033[31mPlease execute './Tools/hc1_config/adb_exe_motor_enable.sh' to enable motor after connected HC1\033[0m"

fi
