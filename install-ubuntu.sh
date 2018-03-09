#!/bin/bash -xe
# On Ubuntu trusty, libuv1-dev isn't available, and libuv-dev refers to the wrong release, 0.10
# Need to get libuv1_1.8.0-1_amd64.deb and libuv1-dev_1.8.0-1_amd64.deb from elsewhere (included in xenial 16.04LTS)
sudo apt-get install -y libssl-dev wget
wget -P /tmp https://mirrors.kernel.org/ubuntu/pool/universe/libu/libuv1 libuv1_1.8.0-1_amd64.deb
wget -P /tmp https://mirrors.kernel.org/ubuntu/pool/universe/libu/libuv1 libuv1-dev_1.8.0-1_amd64.deb
wget -P /tmp https://mirrors.kernel.org/ubuntu/pool/universe/libu/libuv1 libuv1-dbg_1.8.0-1_amd64.deb
sudo apt install /tmp/libuv1_1.8.0-1_amd64.deb
sudo apt install /tmp/libuv1-dev_1.8.0-1_amd64.deb
sudo apt install /tmp/libuv1-dbg_1.8.0-1_amd64.deb
git clone https://github.com/uWebSockets/uWebSockets
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ..
cd ..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
