#!/bin/bash

echo "rplidar usb connection as /dev/rplidar, check it using the command : ls -l /dev | grep ttyUSB"
echo "start copy serial.rules to /etc/udev/rules.d/"
sudo cp serial.rules  /etc/udev/rules.d
echo -e "\nRestarting udev\n"
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish"
