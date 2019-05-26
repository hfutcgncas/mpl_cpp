#!/bin/bash


# ==============================================================
# LD_LIBRARY_PATH
if echo "$LD_LIBRARY_PATH" | grep -q "/usr/local/lib"; then
    echo "lib is contained."
else
    echo "lib is not contained! then export..."
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
    echo export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib  >> ~/.bashrc
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
echo "make yaml-cpp FINISHED"
echo "=========================================="

echo "=========================================="
echo " Start install gtest "
rm -rf $SH_FILE_PATH/googletest/build
mkdir $SH_FILE_PATH/googletest/build
cd $SH_FILE_PATH/googletest/build
cmake ..
make -j8
echo "make gtest FINISHED"
echo "======================================"

cd $SH_FILE_PATH
