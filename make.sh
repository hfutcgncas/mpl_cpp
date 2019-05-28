SH_FILE_PATH=$(pwd)
echo $SH_FILE_PATH
echo "=========================================="
rm -rf $SH_FILE_PATH/build
mkdir $SH_FILE_PATH/build
cd $SH_FILE_PATH/build
cmake -DCMAKE_BUILD_TYPE=Debug ..
# cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
echo "make mpl-cpp FINISHED"
echo "=========================================="
cd $SH_FILE_PATH
