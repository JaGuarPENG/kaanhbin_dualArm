#!/bin/bash
sudo rm -rf /usr/aris
sudo rm -rf /usr/kaanh
sudo rm -rf /usr/kaanhbot

cd ./aris
rm -rf build
mkdir build
cd build
cmake -DETHERCAT=ETHERLAB -DRT_TIMER=XENOMAI3 ..
sudo make install -j8

cd ../../kaanh
rm -rf build
mkdir build
cd build 
cmake ..
sudo make install -j8

cd ../../kaanhbot
rm -rf build
mkdir build
cd build 
cmake ..
sudo make install -j8

cd ../../kaanhbin
rm -rf build
mkdir build
cd build 
cmake ..
make


