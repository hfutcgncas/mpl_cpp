# cdef extern from "robotModel.cpp":
#     pass

from libcpp.string cimport string
from libcpp.map cimport map
from libcpp.vector cimport vector

# Declare the class with cdef
cdef extern from "robotModel.hpp" namespace "RobotModel":
    cdef cppclass RobotModel:
        RobotModel() except +
        RobotModel(string yaml_path) except +
        
        vector[string] getControlableJoints()
        bint setJointValue(string jName, double jValue, bint updateTree)
        bint updateJointsValue(map[string, double] jvMap, bint updateTree)

        bint isCollision()




