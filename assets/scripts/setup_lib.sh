# install glog
mkdir -p /workspace/lib
cd /workspace/lib
git clone https://github.com/google/glog.git
cd glog
git fetch --all --tags
git checkout tags/v0.4.0 -b v0.4.0
mkdir build && cd build
cmake .. && make -j$(nproc)
make install

cd /workspace/lib
git clone https://github.com/gflags/gflags.git
cd gflags
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON && make
make install

cd /workspace/lib
git clone https://github.com/fmtlib/fmt.git
cd fmt
mkdir build && cd build
cmake .. && make -j$(nproc)
make install

# cd /workspace/lib
# git clone --depth 1 --branch v2.3.0 https://github.com/libigl/libigl.git
# cd libigl
# mkdir build && cd build
# cmake .. && make -j$(nproc)
# make install

cd ~
rm -rf /workspace/lib

