#!/usr/bin/env sh

echo 'robot!!!' | sudo -S killall -9 jupyter-notebook

cd ~/etasl_ws/etasl/ws/etasl
sudo rm -rf build
sudo rm -rf devel
ls
sudo update-alternatives --config gcc <<< '0' && sudo update-alternatives --config g++ <<< '0'
catkin_make -DCMAKE_BUILD_TYPE=Release
source ./devel/setup.bash


cd ../etasl-py
sudo rm -rf build
sudo rm -rf devel
ls
sudo update-alternatives --config gcc <<< '1' && sudo update-alternatives --config g++ <<< '1'
catkin_make -DCMAKE_BUILD_TYPE=Release
cd ~
source ~/etasl_ws/etasl/ws/etasl-py/devel/setup.bash
./jupyter_notebook.sh 



