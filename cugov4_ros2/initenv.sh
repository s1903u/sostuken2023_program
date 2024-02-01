#!/bin/bash
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", MODE:="0666", GROUP:="dialout",  SYMLINK+="cugo"' >/etc/udev/rules.d/cugo_pico.rules

service udev reload
sleep 2
service udev restart

