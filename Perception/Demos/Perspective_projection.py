#!interpreter
# -*- coding: utf-8 -*-

"""
Demo of point projection from 3D space to 2D using linear matrix operations.
"""

# ######### Built-in/Generic imports ##########
from typing import List
import numpy as np

__author__ = 'Nico Hessenthaler'
__copyright__ = 'Copyright 2023, HHN - Autonomous Systems: Perception and Situation Understanding'
__credits__ = ['-']
__license__ = '-'
__version__ = '1.0.0'
__maintainer__ = 'Nico Hessenthaler'
__email__ = 'nico.hessenthaler@gmail.com'
__status__ = 'Finished'


# ######### Source code ##########

def project_points(p: List, \
                   f: int,) -> List:
    """
    Function that projects the given point p onto an image plane with the focal length f.

    Args:
        p (List): Point to project onto image plane.
        f (int): Focal length towards the image plane.

    Returns:
        p_image (List): Projected 2D point on image plane.
    """

    # Convert to numpy array for matrix multiplication
    p_homogeneous = np.concatenate((np.array([p]), np.array([[1]])), axis=1).T

    # Create the projection array using focal length
    A = np.array([[f, 0, 0, 0], \
                  [0, f, 0, 0], \
                  [0, 0, 1, 0]])
    
    # Perform the projection
    p_projected = np.dot(A,p_homogeneous)

    # Divide by the homogeneous coordinate
    p_image = [float(p_projected[0] / p_projected[2]), float(p_projected[1] / p_projected[2])]

    return p_image


if __name__ == "__main__":

    # Define parameters for execution
    p = [200, 200, 120]
    f = 50

    # Call the projection
    p_image = project_points(p, f)

    # Print the results
    print(f"\nThe projected image point is: {p_image} pixel\n")

#EOF
