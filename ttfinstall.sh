#!/bin/bash

# meerecompany driver install.
# product VID, PID setting
# TTF_API 1.0.0ver
# root authority command 'ttfinstall.sh'

CURDIR=`pwd`
echo "Current directory is $CURDIR. ttf driver will be installed..."
A=`whoami`

if [ $A != 'root' ]; then
   echo "You have to be root to run this script"
   exit 1;
fi

# Copy rules file
echo "Copy rules file."
cp ttf.rules /etc/udev/rules.d/217-ttf.rules
chmod 777 /etc/udev/rules.d/217-ttf.rules

echo "ttf driver install finish."
