#!/bin/bash

# Copy rules file to location
cp ./10-local.rules /etc/rules.d
# Activate the rules
udevadm control --reload
udevadm trigger

# Copy latency timer script to location
mkdir -p /opt/user_scripts && sudo cp ./set_latency_timer.sh /opt/user_scripts
