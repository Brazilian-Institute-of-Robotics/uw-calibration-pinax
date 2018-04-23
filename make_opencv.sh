#!/bin/bash
GREEN='\033[1;32m'
NC='\033[0m' # no color
virtualEnvName=cv3
nproc=2
user=`id -u -n`

#create virtual environment to ensure use of python 3 bindings (this can be done also for pyhton2)
echo -e "\n${GREEN}>>> Creating OpenCV3 virtual environment${NC}"
source /usr/local/bin/virtualenvwrapper.sh
mkvirtualenv $virtualEnvName
workon $virtualEnvName

#create a build dir for OpenCV and build it
echo -e "\n${GREEN}>>> Configuring CMake files for OpenCV${NC}"
mkdir build && cd build 
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_opencv_java=OFF -D OPENCV_EXTRA_MODULES_PATH=$HOME/opencv_contrib/modules -D INSTALL_C_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON WITH_CUDA=ON -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CUDA_CUDA_LIBRARY=/usr/local/cuda-8.0/targets/x86_64-linux/lib/stubs/libcuda.so -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-8.0 -D CUDA_CUDART_LIBRARY=/usr/local/cuda-8.0/targets/x86_64-linux/lib/libcudart.so -D CUDA_NVCC_FLAGS="-D_FORCE_INLINES" -D CUDA_GENERATION=Auto -D ENABLE_FAST_MATH=1 -D CUDA_FAST_MATH=1 -D WITH_NVCUVID=OFF -D WITH_CUFFT=ON -D WITH_EIGEN=ON -D WITH_IPP=ON .. || exit $?
echo -e "\n${GREEN}>>>Building and installing OpenCV${NC}"
make -j$nproc || exit $?
make install || exit $?
ldconfig || exit $?

echo -e "\n${GREEN}>>> Fixing CUDART library link${NC}"
#link cuda runtime library (necessary in docker)
ln -s /usr/local/cuda-8.0/targets/x86_64-linux/lib/libcudart.so /usr/lib/libcudart.so
sudo ln -s /usr/local/cuda-8.0/targets/x86_64-linux/lib/libcudart.so /usr/lib/libopencv_dep_cudart.so

deactivate
echo -e "\n${GREEN}>>> Finish installing OpenCV in virtual enviroment ${virtualEnvName} ${NC}"


