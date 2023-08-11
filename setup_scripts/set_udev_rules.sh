#!/bin/bash

# Copy rules file to location
cp ./10-local.rules /etc/udev/rules.d/

# Activate the rules
chmod 644 /etc/udev/rules.d/10-local.rules
udevadm control --reload-rules
udevadm trigger

# Copy latency timer script to location
mkdir -p /opt/user_scripts && sudo cp ./set_latency_timer.sh /opt/user_scripts
