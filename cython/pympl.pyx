
# distutils: language = c++

from pympl cimport RobotModel
from libcpp.string cimport string
from libcpp.vector cimport vector

cdef class PyRobotModel:
    cdef RobotModel c_RobotModel  # Hold a C++ instance which we're wrapping

    def __cinit__(self, fileName):
        cdef string fileName_s = fileName
        self.c_RobotModel = RobotModel(fileName_s) 

    
    # def setJointValueMap(self, jv_map, updateTree=False):
    #     cdef string jName_s
    #     for key in jv_map:
    #         jName_s = key
    #         value = jv_map[key]

    #     return self.c_RobotModel.setJointValue( jName_s, value, updateTree )

    def isCollision(self):
        return self.c_RobotModel.isCollision()
    def getControlableJoints(self):
        cdef vector[string] v = self.c_RobotModel.getControlableJoints()
        return [s for s in v]

    def setJointValue(self, jName, value, updateTree=False):
        cdef string jName_s = jName 
        return self.c_RobotModel.setJointValue( jName_s, value, updateTree )
    
    def updateJointsValue(self, js_dict, updateTree=True):
        cdef map[string, double] jv_map
        jv_map.clear()
        for key in js_dict:
            jv_map[key] = js_dict[key]
        self.c_RobotModel.updateJointsValue(jv_map, updateTree)



