rm -rf build
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX="../.." ..
make install
cd ..
rm -rf build
