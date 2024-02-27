import matplotlib.pyplot as plt
import numpy as np
from test_utility import import_lib_module

import_lib_module()
from lib.robot.plot_coordinate import plot_vec, plot_coord


def test_plot_vec():
    ax = plot_vec(np.array([0, 0]), np.array([1, 1]))
    ax = plot_vec(np.array([0, 0, 0]), np.array([1, 1, 1]))
    plt.show()


def test_plot_coord():
    ax = plot_coord(
        np.array([0, 0]),
        np.array([1, 0]),
        np.array([0, 1]),
    )
    ax = plot_coord(
        np.array([0, 0, 0]),
        np.array([1, 0, 0]),
        np.array([0, 1, 0]),
    )
    plt.show()


if __name__ == "__main__":
    # test_plot_vec()
    test_plot_coord()
