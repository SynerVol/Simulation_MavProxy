#!/bin/bash

sudo apt update
sudo apt install git \
  python3-dev \
  python3-pip \
  python3-setuptools \
  python3-venv \
  libxml2-dev \
  libxslt1-dev \
  zlib1g-dev \
  gcc-arm-none-eabi \
  software-properties-common

python3 -m venv venv
. venv/bin/activate
pip install --upgrade pip
pip install MAVProxy \
  pymavlink \
  dronekit \
  future \
  pexpect \
  empy==3.3.4 \
  Pillow \
  opencv-python

pip install MAVProxy --upgrade

###########################################################################"
# Build ardupilot target
###########################################################################
cd ..
git clone --recursive https://github.com/ArduPilot/ardupilot.git
cd ardupilot/ || exit
Tools/environment_install/install-prereqs-ubuntu.sh -y
./modules/waf/waf-light configure --board sitl
./modules/waf/waf-light build --target bin/arducopter
cd ../Simulation_MavProxy/
