import numpy as np
import matplotlib.pyplot as plt
from lib.robot.robot import Link2D, Joint2D, Robot2D, JointType


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


if __name__ == "__main__":
    # test_robot_package()

    joint1 = Joint2D(JointType.revolute, x=0, y=0, theta=90, l=1, name="joint1")
    joint2 = Joint2D(JointType.revolute, x=0, y=1, theta=0, l=2, name="joint2")
    joint3 = Joint2D(JointType.revolute, x=1, y=0, theta=90, l=1, name="joint3")

    robot = Robot2D()
    robot.add_joint(joint1, robot.base)
    robot.add_joint(joint2, joint1)
    robot.add_joint(joint3, robot.base)
    robot.plot()
    print("done")
