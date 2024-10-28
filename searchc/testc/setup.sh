# docker run -it -e POSTGRES_HOST="host.docker.internal" -e POSTGRES_PORT=5432 -e MONGO_HOST="host.docker.internal" -e MONGO_PORT=27017 ubuntu:22.04 bash
# docker run -it ubuntu:24.04 bash



# # Update and install packages in Ubuntu 22.04
# apt update && apt install sudo git g++ curl zip unzip tar wget python3 build-essential libtool bison flex autoconf pkg-config -y

apt-get update && \
apt-get install -y --no-install-recommends \
git \
g++ \
curl \
zip \
unzip \
tar \
wget \
python3 \
build-essential \
libtool \
bison \
flex \
autoconf \
cmake \
pkg-config && \
apt-get clean && \
rm -rf /var/lib/apt/lists/*

apt-get install -y --no-install-recommends
apt-get install sudo
apt-get install git -y
apt-get install g++ -y
apt-get install curl -y
apt-get install zip -y
apt-get install unzip -y
apt-get install tar -y
apt-get install wget -y
apt-get install build-essential -y
apt-get install libtool -y
apt-get install bison -y
apt-get install flex -y
apt-get install autoconf -y
apt-get install pkg-config -y
# apt-get install cmake -y
apt-get install python3 -y

# Please select the geographic area in which you live. Subsequent configuration questions will narrow this down by presenting a list of cities,
# representing the time zones in which they are located.

#   1. Africa  2. America  3. Antarctica  4. Arctic  5. Asia  6. Atlantic  7. Australia  8. Europe  9. Indian  10. Pacific  11. Etc
# Geographic area: 

export DEBIAN_FRONTEND=noninteractive
export TZ=Australia/Melbourne
apt-get install -y python


export DEBIAN_FRONTEND=noninteractive
echo "tzdata tzdata/Areas select Australia" | debconf-set-selections
echo "tzdata tzdata/Zones/Australia select Melbourne" | debconf-set-selections
apt-get install -y python



apt-get update && \
apt-get install -y --no-install-recommends \
build-essential \
git \
curl && \
apt-get clean && \
rm -rf /var/lib/apt/lists/*
    
apt-get update && \
apt-get install -y --no-install-recommends \
libtool \
bison \
flex \
autoconf \
pkg-config && \
apt-get clean && \
rm -rf /var/lib/apt/lists/*


# Install cmake latest version in Ubuntu
# https://apt.kitware.com/
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ noble main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
sudo apt-get update
# echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ noble main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
# echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal-rc main' | sudo tee -a /etc/apt/sources.list.d/kitware.list >/dev/null    

sudo apt update
sudo apt install cmake -y

# Install vcpkg
git clone https://github.com/microsoft/vcpkg.git
cd /vcpkg && ./bootstrap-vcpkg.sh
export VCPKG_ROOT=/vcpkg
export PATH=$VCPKG_ROOT:$PATH

# Configure pkg-config
export PKG_CONFIG_PATH=/vcpkg/installed/x64-linux/lib/pkgconfig:$PKG_CONFIG_PATH

# Install c++ libraries
vcpkg install libpqxx mongo-cxx-driver boost-geometry


# # ---------------------------------------------------------
# # Run compile command
# g++ main.cpp -o test $(pkg-config --cflags --libs libpqxx libpq libmongocxx-static)
# g++ search-path.cpp -o test $(pkg-config --cflags --libs libpqxx libpq libmongocxx-static) -I/vcpkg/installed/x64-linux/include -L/vcpkg/installed/x64-linux/lib
# /workspaces/vicpathfinding/searchc/testc
# # ---------------------------------------------------------

#  pkg-config --cflags --libs libpqxx libpq libmongocxx-static 
# -I/vcpkg/installed/x64-linux/lib/pkgconfig/../../include -DMONGOCXX_STATIC -I/vcpkg/installed/x64-linux/lib/pkgconfig/../../include/mongocxx/v_noabi -DBSONCXX_STATIC -I/vcpkg/installed/x64-linux/lib/pkgconfig/../../include/bsoncxx/v_noabi -fPIC -DMONGOC_STATIC -DUTF8PROC_STATIC -DBSON_STATIC -DUTF8PROC_EXPORTS -L/vcpkg/installed/x64-linux/lib/pkgconfig/../../lib -lpqxx -lpq -L/vcpkg/installed/x64-linux/lib/pkgconfig/../../lib/pkgconfig/../../lib -lpgcommon -lpgport -lm -ldl -pthread -lmongocxx-static -lbsoncxx-static -lmongoc-static-1.0 -lresolv -lssl -lcrypto -lz -lbson-static-1.0 -lrt -lutf8proc

# ---------------------------------------------------------
# Below are old commands
# ---------------------------------------------------------



# curl -OL https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.10.1/mongo-cxx-driver-r3.10.1.tar.gz
# tar -xzf mongo-cxx-driver-r3.10.1.tar.gz
# cd mongo-cxx-driver-r3.10.1/build
# cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_INCLUDEDIR=mongocxx

# cmake .. -DCMAKE_BUILD_TYPE=Release -DMONGOCXX_OVERRIDE_DEFAULT_INSTALL_PREFIX=OFF
# cmake .. -DCMAKE_BUILD_TYPE=Release-DBSONCXX_POLY_USE_BOOST=1-DMONGOCXX_OVERRIDE_DEFAULT_INSTALL_PREFIX=OFF

# cmake --build .
# sudo cmake --build . --target install




# curl -OL https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.11.0/mongo-cxx-driver-r3.11.0.tar.gz
# tar -xzf mongo-cxx-driver-r3.11.0.tar.gz
# cd mongo-cxx-driver-r3.11.0/build

# cmake .. -DCMAKE_BUILD_TYPE=Release -DMONGOCXX_OVERRIDE_DEFAULT_INSTALL_PREFIX=OFF
# cmake --build .
# sudo cmake --build . --target install

# g++ main.cpp -I/usr/local/include/mongocxx -I/usr/local/include/bsoncxx -lpqxx -lpq -lmongocxx

# g++ main.cpp -lpqxx -lpq -lmongocxx

# mv /usr/include/bsoncxx/v_noabi/bsoncxx/* /usr/include/bsoncxx/
# mv /usr/include/mongocxx/v_noabi/mongocxx/* /usr/include/mongocxx/



# version=3.30
# build=2
# # don't modify from here
# mkdir ~/temp
# cd ~/temp
# wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz
# tar -xzvf cmake-$version.$build.tar.gz
# cd cmake-$version.$build/

# ./bootstrap
# make -j$(nproc)
# sudo make install


# mkdir build && cd build && cmake .. && cmake --build .

# g++ main.cpp -o test -I/vcpkg/installed/x64-linux/include -L/vcpkg/installed/x64-linux/lib -lpqxx -lpq -lmongocxx -lbsoncxx
# g++ main.cpp -o test -I/vcpkg/installed/x64-linux/include/pqxx -I/vcpkg/installed/x64-linux/include/mongocxx/v_noabi/mongocxx -I/vcpkg/installed/x64-linux/include/bsoncxx -L/vcpkg/installed/x64-linux/lib -lpqxx -lpq -lmongocxx -lbsoncxx

# g++ main.cpp -o test -I/vcpkg/installed/x64-linux/include -I/vcpkg/installed/x64-linux/include/mongocxx/v_noabi -I/vcpkg/installed/x64-linux/include/bsoncxx/v_noabi -L/vcpkg/installed/x64-linux/lib -lpqxx -lpq /vcpkg/installed/x64-linux/lib/libmongocxx-static.a /vcpkg/installed/x64-linux/lib/libbsoncxx-static.a

# g++ main.cpp -o test \
# -I/vcpkg/installed/x64-linux/include \
# -I/vcpkg/installed/x64-linux/include/mongocxx/v_noabi \
# -I/vcpkg/installed/x64-linux/include/bsoncxx/v_noabi \
# -L/vcpkg/installed/x64-linux/lib \
# -lpq \
# -lpqxx \
# /vcpkg/installed/x64-linux/lib/libmongocxx-static.a \
# /vcpkg/installed/x64-linux/lib/libbsoncxx-static.a \
# /vcpkg/installed/x64-linux/lib/libmongoc-static-1.0.a \
# /vcpkg/installed/x64-linux/lib/libbson-static-1.0.a

# g++ main.cpp -o test \
# -I/vcpkg/installed/x64-linux/include \
# -L/vcpkg/installed/x64-linux/lib \
# -lpq \
# -lpqxx

# root@a175c5325d4b:/app# ls /vcpkg/installed/x64-linux/lib
# libbson-static-1.0.a  libecpg.a         libmongoc-static-1.0.a  libpgport.a   libpqxx-7.9.a  libutf8proc.a
# libbsoncxx-static.a   libecpg_compat.a  libmongocxx-static.a    libpgtypes.a  libpqxx.a      libz.a
# libcrypto.a           liblz4.a          libpgcommon.a           libpq.a       libssl.a       pkgconfig


