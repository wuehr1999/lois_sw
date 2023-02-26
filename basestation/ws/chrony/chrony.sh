#!/bin/bash

/etc/init.d/chrony stop
rm /etc/chrony/chrony.conf
cp chrony.conf /etc/chrony
echo "allow $(cat robot_ip.conf)/24" >> /etc/chrony/chrony.conf
/etc/init.d/chrony start
