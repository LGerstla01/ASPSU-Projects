#!interpreter
# -*- coding: utf-8 -*-

"""
Verification script for perspective_projection demo.
"""

# ######### Built-in/Generic imports ##########
from typing import Dict
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

__author__ = 'Nico Hessenthaler'
__copyright__ = 'Copyright 2025, HHN - Autonomous Systems: Perception and Situation Understanding'
__credits__ = ['-']
__license__ = '-'
__version__ = '1.0.0'
__maintainer__ = 'Nico Hessenthaler'
__email__ = 'nico.hessenthaler@gmail.com'
__status__ = 'Finished'


# ######### Source code ##########

def verify_projection(points: Dict) -> None:
    """
    Function that projects the given point p onto an image plane with the camera intrinsic parameters.

    Args:
        points (Dict): Dictionary containing the 3 points to plot.

    Returns:
        ():
    """

    # Unpack points
    x = [point[0] for point in points.values()]
    y = [point[1] for point in points.values()]
    z = [point[2] for point in points.values()]
    labels = list(points.keys())

    # 3D Plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot points in figure
    ax.scatter(x, y, z, c='blue', marker='o', s=100)

    # Label the points
    for i, label in enumerate(labels):
        ax.text(x[i], y[i], z[i], label, color='red')

    # Set axis descriptions
    ax.set_xlabel('Z-axis')
    ax.set_ylabel('X-axis')
    ax.set_zlabel('Y-axis')

    # Set title
    plt.title('Verification of perspective projection')

    # Show 3D plot
    plt.show()

    return


if __name__=="__main__":

    points = {
    "original_point": (-120.0, 200.0, 200.0),
    "projected_point": (-50.0, 83.33, 83.33),
    "center_of_projection": (0.0, 0.0, 0.0)
    }

    verify_projection(points)
