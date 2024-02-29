import matplotlib.pyplot as plt
from lib.robot.robot import Robot2D
from lib.robot.joint import Joint2D, JointType

joint1 = Joint2D(j_type=JointType.revolute, x=0, y=0, theta=0, l=2, name="joint1")
joint2 = joint1.append_joint(JointType.revolute, 0, 1, "joint2")

robot = Robot2D.from_joint_list([joint1, joint2])
robot.plot()
