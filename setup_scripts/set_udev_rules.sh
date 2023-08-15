#!/bin/bash

# Copy rules file to location
cp ./10-local.rules /etc/udev/rules.d/

# Activate the rules
chmod 644 /etc/udev/rules.d/10-local.rules
udevadm control --reload-rules
udevadm trigger
