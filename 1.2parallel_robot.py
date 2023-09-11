import numpy as np
import matplotlib.pyplot as plt
from lib.robot.robot import KinematicChain, Joint2D, Robot2D, JointType


def test_robot_package():
    fig, ax = plt.subplots()
    joint1 = Joint2D(JointType.revolute, x=1, y=1, theta=90, l=1, name="joint1")
    joint1.plot(ax)

    ax.set_aspect("equal")
    ax.grid()

    joint3 = Joint2D(JointType.revolute, x=0, y=1, theta=90, l=1, name="joint3")
    joint4 = Joint2D(JointType.revolute, x=0, y=2, theta=45, l=1, name="joint4")
    robot = Robot2D([joint3, joint4])
    robot._plot(ax)
    plt.show()


def test_node():
    kc = KinematicChain()
    kc.add_node("node1", "BASE")
    kc.add_node("node2", "node1")
    kc.add_node("node3", "node2")
    kc.add_node("node4", "node2")
    kc.add_node("node5", "node4")
    kc.add_node("node6", "node4")
    kc.add_node("node7", "node6")
    s3 = kc.get_node_child_to_end("node3")
    s4 = kc.get_node_child_to_end("node4")
    s2 = kc.get_node_child_to_end("node2")
    s1 = kc.get_node_child_to_end("node1")
    print("s1", s1, "s2", s2, "s3", s3, "s4", s4)


def test_robot():
    joint1 = Joint2D(JointType.revolute, x=0, y=0, theta=90, l=1, name="joint1")
    joint2 = Joint2D(JointType.revolute, x=0, y=1, theta=0, l=2, name="joint2")
    joint3 = Joint2D(JointType.revolute, x=1, y=0, theta=90, l=1, name="joint3")

    robot = Robot2D()
    robot.add_joint(joint1, robot.base)
    robot.add_joint(joint2, joint1)
    robot.add_joint(joint3, robot.base)
    robot.plot()

    robot_struct = robot.get_node_child_to_end(joint1.name)
    print(robot_struct)


if __name__ == "__main__":
    # test_robot_package()
    # test_node()
    test_robot()

    print("done")
