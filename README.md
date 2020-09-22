# eris
Hand-eye calibration from point clouds

## Installation

### Install dependencies
```bash
# GCC
sudo apt-get install g++
# CMake
sudo apt-get install cmake cmake-curses-gui
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# Ninja
sudo apt-get install ninja-build
# Pip3
sudo apt-get install python3-pip
```

### Download and install Ceres
```bash
wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
tar -xf ceres-solver-1.14.0.tar.gz 
mkdir -p ceres-solver-1.14.0/build
cd ceres-solver-1.14.0/build
cmake -GNinja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF ..
cmake --build .
cmake --build . --target install 
```

### Clone and install
```bash
git clone https://github.com/tingelst/eris-calibration.git
pip3 install eris-calibration
```