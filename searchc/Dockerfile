FROM ubuntu:24.04

# Set environment variables
ENV POSTGRES_HOST="host.docker.internal"
ENV POSTGRES_PORT=5432
ENV MONGO_HOST="host.docker.internal"
ENV MONGO_PORT=27017

# Install necessary tools
RUN apt-get update
RUN apt-get install -y sudo
RUN apt-get install -y wget
RUN apt-get install -y git
RUN apt-get install -y g++
RUN apt-get install -y curl
RUN apt-get install -y zip
RUN apt-get install -y unzip
RUN apt-get install -y tar
RUN apt-get install -y libtool
RUN apt-get install -y bison
RUN apt-get install -y flex
RUN apt-get install -y autoconf
RUN apt-get install -y build-essential
RUN apt-get install -y ca-certificates
RUN apt-get install -y gpg
RUN apt-get install -y pkg-config
RUN apt-get install -y python3

# Install CMake

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null

RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ noble main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null

RUN sudo apt-get update 

RUN sudo apt-get install -y cmake

# Install vcpkg
RUN git clone https://github.com/microsoft/vcpkg.git

# Bootstrap vcpkg
RUN cd /vcpkg && ./bootstrap-vcpkg.sh

# Set environment variables
ENV VCPKG_ROOT="/vcpkg"
ENV PATH="${VCPKG_ROOT}:${PATH}"

# Configure pkg-config
ENV PKG_CONFIG_PATH="${VCPKG_ROOT}/installed/x64-linux/lib/pkgconfig:${PKG_CONFIG_PATH}"

# Install necessary packages
RUN vcpkg install libpqxx mongo-cxx-driver boost-geometry

# Set the default command to bash
CMD ["bash"]
