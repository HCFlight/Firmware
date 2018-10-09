#!/bin/bash

dirPath=`dirname $0`
adbState=`adb get-state`
device="device"

if [ $adbState == $device ];then

adb push $dirPath/hc1.main.mix  /usr/share/data/adsp
echo -e "\033[32mHC1.main.mix upload success !\033[0m"

else

echo -e "\033[31mHC1 mixer upload failed!"
echo "HC1 device disconnected"
echo -e "\033[31mPlease execute './Tools/hc1_config/motor_mixer_upload.sh' to enable motor after connected HC1\033[0m"

fi
