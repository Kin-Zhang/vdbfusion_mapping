for glog, please just run the script directly

```bash
cd [YOUR VDBMAPPING WS]/src
sudo chmod +x ./assets/scripts/setup_lib.sh
./assets/scripts/setup_lib.sh

sudo apt install ros-$ROS_DISTRO-tf2-sensor-msgs
```

Then just install the OpenVDB is enough, but if you want to save the rgb mesh or visualize real-time, Open3D is needed

# OpenVDB

## Dependencies

system
```bash
sudo apt-get update && sudo apt-get install -y libblosc-dev \
    libboost-iostreams-dev \
    libboost-system-dev \
    libboost-system-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*
```

boost 1.65+ install 1.70, please note!!! this one may have conflict with your env, be careful!
```bash
wget https://boostorg.jfrog.io/artifactory/main/release/1.70.0/source/boost_1_70_0.tar.gz \
   && tar -zxvf boost_1_70_0.tar.gz \
   && cd boost_1_70_0 \
   && ./bootstrap.sh --prefix=/usr/local/boost_1_70_0 && ./b2 && ./b2 install
```

tcc
```bash
git clone https://gitee.com/li-ming-golang/tbb \
   && cd tbb && cd build && cmake .. && make -j4 && sudo make install
```

## Source code

openvdb from source
```bash
git clone --depth 1 https://github.com/nachovizzo/openvdb.git -b nacho/vdbfusion \
    && cd openvdb \
    && mkdir build && cd build \
    && cmake  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DUSE_ZLIB=OFF .. \
    && make -j$(nproc) all install
```

reference from: [http://gitlab.ram-lab.com/ramlab_dataset_sensor/mapping_codebase/vdbmapping](http://gitlab.ram-lab.com/ramlab_dataset_sensor/mapping_codebase/vdbmapping)

more official install and problems, please click [here: https://www.openvdb.org/documentation/doxygen/build.html](https://www.openvdb.org/documentation/doxygen/build.html)

## Note Problems I met

problem may occur in melodic/Ubuntu 18.04

after install boost 1.70.0, it still miss need one more action to link together:

```bash
sudo ln -s /usr/local/boost_1_70_0/lib/libboost_iostreams.so.1.70.0 /usr/local/lib/libboost_iostreams.so.1.70.0
```

# Open3D
mainly for save the rgb mesh and visualize real-time, if you don't want to install, please just comment the code related to this one, for save mesh, you can uncomment the igl
build from source
```bash
git clone --depth 1 --branch v0.16.0 https://gitcode.net/mirrors/isl-org/Open3D.git && cd Open3D \
    && util/install_deps_ubuntu.sh && mkdir build && cd build && cmake .. \
    && make -j$(nproc) \
    && make -j$(nproc) all install \
    && cd / \
    && rm -rf /Open3D
```

or use the binary lib but need to cp/ln the lib, there are four steps after download here: [https://github.com/isl-org/Open3D/releases](https://github.com/isl-org/Open3D/releases)
```bash
sudo cp -r ~/Desktop/Open3D/include/open3d /usr/local/include
sudo cp -r ~/Desktop/Open3D/lib/cmake/Open3D /usr/local/lib/cmake
sudo cp -r ~/Desktop/Open3D/libOpen3D.so /usr/local/lib
```