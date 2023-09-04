import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Literal

LINK2D_PAIR = Tuple["Link2D", "Link2D"]
REVOLUTE_OR_PRISMATIC = Literal["revolute", "prismatic"]


class Link2D:
    def __init__(
        self,
        x: float,
        y: float,
        theta: float = 0,
        l: float = 1,
        label: str = "",
        rad: bool = False,
    ):
        self.x = x
        self.y = y
        if rad:
            self.theta = theta
        else:
            self.theta = np.deg2rad(theta)
        self.length = l
        self.label = label

    @property
    def end_x(self):
        return self.x + self.length * np.cos(self.theta)

    @property
    def end_y(self):
        return self.y + self.length * np.sin(self.theta)

    def _plot_start_point(self, ax, color="black", markersize=5):
        ax.plot(self.x, self.y, "o", color=color, markersize=markersize)
        if self.label != "":
            ax.annotate(
                self.label,
                (self.x, self.y),
                color="black",
                fontsize=8,
                weight="heavy",
                horizontalalignment="center",
                verticalalignment="center",
            )

    def _plot_end_point(self, ax, color="black"):
        ax.plot(self.end_x, self.end_y, "o", c=color, markersize=5)

    def _plot_link(self, ax, color="black"):
        ax.plot([self.x, self.end_x], [self.y, self.end_y], color=color)

    def plot(self, ax):
        self._plot_start_point(ax)
        self._plot_end_point(ax)
        self._plot_link(ax)

    def new_l(self, l):
        self.length = l

    def new_theta(self, theta):
        self.theta = theta

    def new_label(self, label):
        self.label = label


class Joint2D:
    def __init__(self, j_type: REVOLUTE_OR_PRISMATIC, link: Link2D, label: str = ""):
        self.j_type = j_type
        self.link = link
        self.link.new_label(label)

    def plot(self, ax, color="red"):
        self.link.plot(ax)
        self.link._plot_start_point(ax, color=color, markersize=10)

    def new_joint_value(self, j_value: float):
        if self.j_type == "revolute":
            self.link.new_theta(j_value)
        elif self.j_type == "prismatic":
            self.link.new_l(j_value)

    def new_pos(self, x, y):
        self.link.x = x
        self.link.y = y


class Robot2D:
    def __init__(self, joints: list[Joint2D], base: Tuple[float, float] = (0, 0)):
        self.joints = joints
        self._check_joints()

    def _check_joints(self):
        for i, joint in enumerate(self.joints):
            if i > 0:
                prev_joint = self.joints[i - 1]
                error_msg = (
                    f"The joints must be connected to each other. "
                    f"{joint.link.label} started at ({joint.link.x},{joint.link.y}), "
                    f"but the previous joint ended at ({prev_joint.link.end_x},{prev_joint.link.end_y})."
                )
                loc_error = (prev_joint.link.end_x - joint.link.x) ** 2 + (
                    prev_joint.link.end_y - joint.link.y
                ) ** 2
                assert loc_error < 1e-6, f"{error_msg}"

    @property
    def colors(self):
        n_joints = len(self.joints)
        cmap = plt.get_cmap("hsv")
        norm = plt.Normalize(vmin=0, vmax=n_joints)
        colors = cmap(norm(range(n_joints)))
        return colors

    def _set_joint_pose(self, i_joint: int, pose: float):
        self.joints[i_joint].new_joint_value(pose)

    def plot(self, ax):
        for joint, c in zip(self.joints, self.colors):
            joint.plot(ax, color=c)


if __name__ == "__main__":
    fig, ax = plt.subplots()
    link1 = Link2D(0, 0, 45, 1)
    link1.plot(ax)

    link2 = Link2D(1, 1, 90, 1)
    joint1 = Joint2D("revolute", link2, label="joint1")
    joint1.plot(ax)

    ax.set_aspect("equal")
    ax.grid()

    link3 = Link2D(0, 1, 90, 1)
    link4 = Link2D(0, 2, 45, 1)
    joint3 = Joint2D("revolute", link3, label="joint3")
    joint4 = Joint2D("revolute", link4, label="joint4")
    robot = Robot2D([joint3, joint4])
    robot.plot(ax)
    plt.show()
