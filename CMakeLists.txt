PROJECT(mpl_cpp)

cmake_minimum_required(VERSION 3.2.0)
add_definitions(-std=c++11)
add_definitions(-g -o2)

# use opencv
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/share/OpenCV")


# headers
include_directories( ${PROJECT_SOURCE_DIR}/include 
                     ${PROJECT_SOURCE_DIR}/Thirdparty/yaml-cpp/include # yaml-cpp
                     ${PROJECT_SOURCE_DIR}/Thirdparty/boost_1_7_0  # boost
                   )

#lib link
link_directories( ${PROJECT_SOURCE_DIR}/Thirdparty/yaml-cpp/build # yaml-cpp
                )

#sources
set(SRC  ${PROJECT_SOURCE_DIR}/src/main.cpp  ${PROJECT_SOURCE_DIR}/src/robotModel.cpp)
# set(SRC_test  ${PROJECT_SOURCE_DIR}/src/main_test.cpp)

#build so
# add_library(plate_recognition SHARED ${SRC})
# target_link_libraries(plate_recognition ${OpenCV_LIBS})
# target_link_libraries(plate_recognition -lxxx -lxxx   -lpthread -lm -lstdc++)


ADD_EXECUTABLE(main ${SRC})
target_link_libraries(main  yaml-cpp)


# ADD_EXECUTABLE(main_test ${SRC_test})
