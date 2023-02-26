#!/bin/bash

/etc/init.d/chrony stop
rm /etc/chrony/chrony.conf
cp chrony.conf /etc/chrony
echo "server $(cat basestation_ip.conf) iburst prefer" >> /etc/chrony/chrony.conf
/etc/init.d/chrony start
while true
do
  clear
  chronyc sources -v
  sleep 2
done
