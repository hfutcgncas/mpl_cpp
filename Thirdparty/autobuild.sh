#!/bin/bash

rm -rf libs
mkdir libs

# ==============================================================
# LD_LIBRARY_PATH
if echo "$LD_LIBRARY_PATH" | grep -q "/usr/local/lib"; then
    echo "lib is contained."
else
    echo "lib is not contained! then export..."
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
    echo export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib >>~/.bashrc
fi

# ==============================================================
SH_FILE_PATH=$(pwd)
echo $SH_FILE_PATH
echo "=========================================="
echo " Start install yaml-cpp "
rm -rf $SH_FILE_PATH/yaml-cpp/build
mkdir $SH_FILE_PATH/yaml-cpp/build
cd $SH_FILE_PATH/yaml-cpp/build
cmake ..
make -j8
cp $SH_FILE_PATH/yaml-cpp/build/libyaml-cpp.a $SH_FILE_PATH/libs/
cp $SH_FILE_PATH/yaml-cpp/build/libyaml-cpp.so* $SH_FILE_PATH/libs/
echo "make yaml-cpp FINISHED"
echo "=========================================="

echo "=========================================="
echo " Start install gtest "
rm -rf $SH_FILE_PATH/googletest/build
mkdir $SH_FILE_PATH/googletest/build
cd $SH_FILE_PATH/googletest/build
cmake ..
make -j8
cp $SH_FILE_PATH/googletest/build/lib/* $SH_FILE_PATH/libs/
echo "make gtest FINISHED"
echo "======================================"

echo "=========================================="
echo " Start install libccd "
rm -rf $SH_FILE_PATH/libccd/build
mkdir $SH_FILE_PATH/libccd/build
cd $SH_FILE_PATH/libccd/build
cmake -G "Unix Makefiles" ..
make -j8
cp $SH_FILE_PATH/libccd/build/src/libccd* $SH_FILE_PATH/libs/
echo "make libccd FINISHED"
echo "======================================"

echo "=========================================="
echo " Start install octomap "
rm -rf $SH_FILE_PATH/octomap/build
mkdir $SH_FILE_PATH/octomap/build
cd $SH_FILE_PATH/octomap/build
cmake -BUILD_OCTOVIS_SUBPROJECT=OFF ..
make -j8
cp -r $SH_FILE_PATH/octomap/lib/* $SH_FILE_PATH/libs/
echo "make octomap FINISHED"
echo "======================================"

echo "=========================================="
echo " Start install fcl "
rm -rf $SH_FILE_PATH/fcl/build
mkdir $SH_FILE_PATH/fcl/build
cd $SH_FILE_PATH/fcl/build
cmake ..
make -j8
cp -r $SH_FILE_PATH/fcl/build/lib/libfcl* $SH_FILE_PATH/libs/
echo "make fcl FINISHED"
echo "======================================"

cd $SH_FILE_PATH
