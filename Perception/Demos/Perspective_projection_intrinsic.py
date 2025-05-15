#!interpreter
# -*- coding: utf-8 -*-

"""
Demo of point projection from 3D space to 2D using camera intrinsic matrix.
"""

# ######### Built-in/Generic imports ##########
from typing import List, Type
import numpy as np
import cv2
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

def project_point(p: List) -> List:
    """
    Function that projects the given point p onto an image plane with the camera intrinsic parameters.

    Args:
        p (List): Point to project onto image plane.

    Returns:
        p_image (List): Projected 2D point on image plane.
    """

    # Create the matrixs of camera intrinsic parameters, considering homogeneous coordinate
    K = np.array([[1606.6, 0.0, 672.3, 0],\
                  [0.0, 1600.3, 331.5, 0],\
                  [0.0, 0.0, 1.0, 0]])
    
    # Perform the projection
    p_projected = np.dot(K, p)

    # Divide by the homogeneous coordinate
    u = int(p_projected[0] / p_projected[2])
    v = int(p_projected[1] / p_projected[2])

    return [u, v]


def visualize_point(p: List, 
                    image: Type[np.ndarray]) -> None:
    """
    Function that visualizes a given point on an image.

    Args:
        p (List): Point to project onto image plane.
        image (np.ndarray): Numpy array of image to visualize the point on

    Returns:
        ():
    """
    
    cv2.circle(image,(p[0], p[1]), 3, (0,0,255), -1)
    cv2.imshow("Frame", image)
    cv2.waitKey()
    
    return


if __name__=="__main__":

    # Definition of the point according to DIN 70000 
    p_din = [40.673, -0.144, -1.563]

    # Definition according to the given camera coordinate system, add homogeneous coordinate
    p_cam = [-p_din[1], -p_din[2], p_din[0], 1]

    # Perform the projection
    p_img = project_point(p_cam)

    # Read the image
    image_path = f"C:\\Users\\lukas\\Dokumente\\Studium\\Master\\2. Semester\\Fächer\\Autonomous Systems Perception and Situation Understanding\\ASPSU-Projects\\Perception\\Tasks\\Images\\test_image.png"
    
    image = \
        plt.imread(image_path)

    # Visualize the point in the image
    print(p_img)
    visualize_point(p_img, image)
