from pympl import PyRobotModel
from timeit import timeit 



a = PyRobotModel("/home/liujianran/temp/mpl_cpp/mechmind_yaml_model.yaml")

a.setJointValue("joint_1", 0, False)
a.setJointValue("joint_2", 1.27, True)

j_map = dict()
j_map["joint_1"] = 0
j_map["joint_2"] = 1.27
j_map["joint_3"] = 0
j_map["joint_4"] = 0
j_map["joint_5"] = 0
j_map["joint_6"] = 0

a.updateJointsValue(j_map)

print a.isCollision()
print a.getControlableJoints()
print type(a.getControlableJoints()[0])