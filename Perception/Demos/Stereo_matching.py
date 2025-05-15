#!interpreter
# -*- coding: utf-8 -*-

"""
Demo of template matching using normalized cross correlation in stereo images.
"""

# ######### Built-in/Generic imports ##########

from typing import Type, Tuple
from copy import deepcopy
import numpy as np
import cv2

__author__ = 'Nico Hessenthaler'
__copyright__ = 'Copyright 2023, HHN - Autonomous Systems: Perception and Situation Understanding'
__credits__ = ['-']
__license__ = '-'
__version__ = '1.0.0'
__maintainer__ = 'Nico Hessenthaler'
__email__ = 'nico.hessenthaler@gmail.com'
__status__ = 'Finished'

# ######### Variables ##########

ALPHA_VALUES = [1, 1, 1, 1.5, 0.5]
BETA_VALUES = [0, 100, -20, 0, 0]
FRAME_NAMES = ['Matched Image', 'Matched Image Brighter', \
               'Matched Image Darker', 'Matched Image Higher Contrast', \
               'Matched Image Lower Contrast']


# ######### Source code ##########

def adjust_brightness_contrast(alpha: int, \
                               beta: int, \
                               img: Type[np.array]) -> Type[np.array]:
    """
    Function that adjusts contrast alpha and brightness beta for a given image.

    Args:
        alpha (int): Contrast value.
        beta (int): Brightness value.
        img (np.array): Original image to be adjusted.

    Returns:
        adjusted_img (np.array): Image with adjusted contrast and brightness.
    """

    # Convert to numpy array for matrix multiplication
    adjusted_img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

    return adjusted_img


def match_template(template: Type[np.array], \
                   img: Type[np.array]) -> Tuple:
    """
    Function that matches a template on a given image.

    Args:
        template (int): Template to search in the image.
        img (np.array): Image to match the template in.

    Returns:
        max_location (Tuple): Pixel position of the matched template top left corner.
    """

    # Perform normalized cross-correlation using cv2.matchTemplate()
    result = cv2.matchTemplate(img, template, cv2.TM_CCORR_NORMED)

    # Get the location of the maximum value in the result array
    _, _, _, max_location = cv2.minMaxLoc(result)

    return max_location


def draw_rectangle(img: Type[np.array], \
                   max_location: Tuple, \
                   tw: int, \
                   th: int) -> None:
    """
    Function that draws a rectangle on an image.

    Args:
        img (np.array): Image to match the template in.
        max_location (Tuple): Pixel position of the matched template top left corner.
        tw (int): Width of the template image.
        th (int): Height of the template image.

    Returns:
        ():
    """

    # Draw a rectangle around the matched region
    top_left = max_location
    bottom_right = (top_left[0] + tw, top_left[1] + th)
    cv2.rectangle(img, top_left, bottom_right, (0, 255, 0), 1)

    return 


def show_image(img: Type[np.array], \
               frame_name: str) -> None:
    """
    Function that shows an image.

    Args:
        img (np.array): Image to match the template in.
        frame_name (str): Name of the image frame.

    Returns:
        ():
    """

    # Display the result
    cv2.imshow(frame_name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return


if __name__ == "__main__":

    # Load the source and template images
    img = cv2.imread('./images/source_image.jpg')
    template = cv2.imread('./images/template_image.jpg')
    

    # Get the width and height of the template image
    tw, th = template.shape[:2]

    # Loop over all relevant cases:
    for alpha, beta, frame_name in zip(ALPHA_VALUES, BETA_VALUES, FRAME_NAMES):

        # Get a new instance of the image
        img_orig = deepcopy(img)

        # Adjust brightness and contrast according to the definition
        adjusted_image = adjust_brightness_contrast(alpha, beta, img_orig)

        # Match the template for the given image
        max_location = match_template(template, adjusted_image)

        # Draw the result on the image:
        draw_rectangle(adjusted_image, max_location, tw, th)

        # Show the result
        show_image(adjusted_image, frame_name)

#EOF
