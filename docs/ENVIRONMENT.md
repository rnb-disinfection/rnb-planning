# Basic Environment Setup (GCC and Basic packages)
* Turn off "Fast boot" & "Secure boot" on Bios
* Ubuntu 18.04  
* install gcc7 & gcc5
```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test && sudo apt-get update && sudo apt-get install gcc-7 g++-7 gcc-7-multilib g++-7-multilib \
&& sudo add-apt-repository ppa:ubuntu-toolchain-r/test && sudo apt-get update && sudo apt-get install gcc-5 g++-5 gcc-5-multilib g++-5-multilib  
```
* set gcc alternative versions:  
```bash
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 20 \
&& sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 40 \
&& sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 20 \
&& sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 40  
```
* check gcc/g++ alternative version  
```bash
sudo update-alternatives --display gcc && sudo update-alternatives --display g++  
```
* pip
```bash
sudo apt-get install python-pip && pip install --upgrade pip \
&& sudo apt-get install python3-pip && pip3 install --upgrade pip \
&& pip install setuptools==41.0.0 \
&& pip3 install setuptools==41.0.0  
```