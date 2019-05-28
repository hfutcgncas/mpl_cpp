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
                     ${PROJECT_SOURCE_DIR}/Thirdparty/eigen        # eigen 
                     ${PROJECT_SOURCE_DIR}/Thirdparty/googletest/googletest/include # gtest
                   )

#lib link
link_directories( ${PROJECT_SOURCE_DIR}/Thirdparty/yaml-cpp/build # yaml-cpp
                  ${PROJECT_SOURCE_DIR}/Thirdparty/googletest/build/lib # gtest
                )

#sources
set(SRC  ${PROJECT_SOURCE_DIR}/src/main.cpp  ${PROJECT_SOURCE_DIR}/src/robotModel.cpp)

#build so
# add_library(plate_recognition SHARED ${SRC})
# target_link_libraries(plate_recognition ${OpenCV_LIBS})
# target_link_libraries(plate_recognition -lxxx -lxxx   -lpthread -lm -lstdc++)


# set(EXECUTABLE_OUTPUT_PATH ../build/bin)



ADD_EXECUTABLE(main ${SRC})
target_link_libraries(main  yaml-cpp)


# ADD_EXECUTABLE(main_test ${SRC_test})


# set(EXECUTABLE_OUTPUT_PATH ../build/test)
set(TEST_SRC  ${PROJECT_SOURCE_DIR}/test/test_main.cpp
              ${PROJECT_SOURCE_DIR}/test/test_robotModel.cpp
              ${PROJECT_SOURCE_DIR}/test/test_tfGraph.cpp
              )

ADD_EXECUTABLE(utest ${TEST_SRC} ${PROJECT_SOURCE_DIR}/src/robotModel.cpp)
target_link_libraries(utest  yaml-cpp gtest gtest_main pthread)



# add_executable(runUnitTests
#         gtest.cpp
#         test1.cpp)

# target_link_libraries(runUnitTests
#         gtest
#         pthread)
