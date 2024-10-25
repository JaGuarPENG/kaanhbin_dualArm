#!/bin/bash
. /opt/poky/3.1.20/environment-setup-aarch64-poky-linux

rm ./aris/CMakeLists.txt
cp ./kaanhbin/CMakeLists_aris.txt ./aris/CMakeLists.txt
cd ./aris
rm -rf build
mkdir build
cd build
cmake -DCMAKE_CXX_COMPILER=aarch64-poky-linux-g++ -DCMAKE_C_COMPILER=aarch64-poky-linux-gcc -DRT_TIMER=RT_PREEMT -DETHERCAT=ETHERLAB ..
sudo make uninstall
sudo make install -j8

cd ../../kaanh
rm -rf build
mkdir build
cd build 
cmake -DMYIR=ON ..
sudo make uninstall
sudo make install -j8

cd ../../kaanhbot
rm -rf build
mkdir build
cd build 
cmake -DMYIR=ON ..
sudo make uninstall
sudo make install -j8

cd ../../kaanhbin
rm -rf build
mkdir build
cd build 
cmake -DMYIR=ON ..
make -j8

cd ../../
sudo chmod -R +666 *

