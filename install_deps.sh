#!/bin/bash
echo "Install development deps..."
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install build-essential automake autoconf libtool yasm git gnupg flex bison gperf curl tofrodos u-boot-tools device-tree-compiler libncurses5-dev zip gzip libzip-dev
sudo apt-get install libopencv-dev python-opencv
echo "Done!"