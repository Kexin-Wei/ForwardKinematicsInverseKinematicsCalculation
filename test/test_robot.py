import numpy as np
import matplotlib.pyplot as plt
from test_utility import import_lib_module

import_lib_module()

from lib.robot.robot import Robot2D
from lib.robot.kinematic_chain import KinematicChain
from lib.robot.joint import Joint2D, JointType


def test_2d_serial_robot():
    joint1 = Joint2D(
        j_type=JointType.REVOLUTE, x=0, y=0, theta=0, l=2, name="joint1"
    )
    joint2 = joint1.append_joint(JointType.REVOLUTE, 0, 1, "joint2")
    robot = Robot2D.from_joint_list([joint1, joint2])
    robot.plot()


def test_robot_package():
    fig, ax = plt.subplots()
    joint1 = Joint2D(JointType.REVOLUTE, x=1, y=1, theta=90, l=1, name="joint1")
    joint1.plot(ax)

    ax.set_aspect("equal")
    ax.grid()

    joint3 = Joint2D(JointType.REVOLUTE, x=0, y=1, theta=90, l=1, name="joint3")
    joint4 = Joint2D(JointType.REVOLUTE, x=0, y=2, theta=45, l=1, name="joint4")
    robot = Robot2D()
    robot.add_joint(joint3, robot.base)
    robot.add_joint(joint4, joint3)
    robot.plot(ax=ax)


def test_node():
    kc = KinematicChain()
    kc.add_node_to_parent("node1", "BASE")
    kc.add_node_to_parent("node2", "node1")
    kc.add_node_to_parent("node3", "node2")
    kc.add_node_to_parent("node4", "node2")
    kc.add_node_to_parent("node5", "node4")
    kc.add_node_to_parent("node6", "node4")
    kc.add_node_to_parent("node7", "node6")
    s3 = kc.get_node_child_to_end("node3")
    s4 = kc.get_node_child_to_end("node4")
    s2 = kc.get_node_child_to_end("node2")
    s1 = kc.get_node_child_to_end("node1")
    print("s1", s1, "s2", s2, "s3", s3, "s4", s4)


def get_2d_serial_robot():
    joint1 = Joint2D(JointType.REVOLUTE, x=0, y=0, theta=90, l=1, name="joint1")
    joint2 = Joint2D(JointType.REVOLUTE, x=0, y=1, theta=0, l=2, name="joint2")
    joint3 = Joint2D(JointType.REVOLUTE, x=1, y=0, theta=90, l=1, name="joint3")

    robot = Robot2D()
    robot.add_joint(joint1, robot.base)
    robot.add_joint(joint2, joint1)
    robot.add_joint(joint3, robot.base)
    return robot


def get_2d_parallel_robot():
    joint1 = Joint2D(JointType.REVOLUTE, x=0, y=0, theta=90, l=1, name="joint1")
    joint2 = Joint2D(JointType.REVOLUTE, x=0, y=1, theta=0, l=2, name="joint2")
    joint3 = Joint2D(JointType.REVOLUTE, x=1, y=0, theta=90, l=1, name="joint3")

    robot = Robot2D()
    robot.add_joint(joint1, robot.base)
    robot.add_joint(joint2, joint1)
    robot.add_joint(joint3, robot.base)
    robot.add_parallel_joint(
        "joint3", "joint2", [joint3.xn, joint3.yn], JointType.REVOLUTE
    )
    return robot


def test_robot():
    robot = get_2d_parallel_robot()
    robot.plot()
    print(robot.struct)
    # robot.forward()


def test_robot_kinematics():
    robot1 = get_2d_serial_robot()
    robot1.forward()
    robot2 = get_2d_parallel_robot()
    robot2.forward()


def test_workspace():
    robot = get_2d_serial_robot()
    print(robot.struct)
    print(robot.joint_range)
    print(robot.get_joint_range_per_chain)
    robot.plot_workspace()


if __name__ == "__main__":
    test_2d_serial_robot()
    # test_robot_package()
    # test_node()
    # test_robot()
    # TODO finish workspace code by using dual quaternion
    # test_workspace()
    # test_robot_kinematics()

    print("done")
