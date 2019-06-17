from distutils.core import setup, Extension

from Cython.Build import cythonize


include_dirs_list = [
                    "../include",
                    "../Thirdparty/libccd/src", #libccd
                    "../Thirdparty/libccd/build/src", #libccd
                    "../Thirdparty/yaml-cpp/include", # yaml-cpp
                    "../Thirdparty/boost_1_7_0", # boost
                    "../Thirdparty/eigen", # eigen 
                    "../Thirdparty/googletest/googletest/include", # gtest
                    "../Thirdparty/octomap/octomap/include", # octomap ? 
                    "../Thirdparty/fcl/build/include", # fcl
                    "../Thirdparty/fcl/include", # fcl

]


setup(ext_modules = cythonize(Extension(
           "pympl",                                # the extension name
           sources=["pympl.pyx"],       # the Cython source and additional C++ source files
           language="c++",                        # generate and compile C++ code
           include_dirs=include_dirs_list,
           library_dirs=["../build"],
           libraries=["mpl"],
           extra_compile_args=["-std=c++11"]
      )))