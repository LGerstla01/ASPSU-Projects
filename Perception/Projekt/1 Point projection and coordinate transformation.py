import cv2
import numpy as np


image1_path = 'Perception\Projekt\Additional_files\Stereo_images\ConstructionSiteLeft\image0110_c0.pgm'
image2_path = 'Perception\Projekt\Additional_files\Stereo_images\ConstructionSiteRight\image0110_c1.pgm'

image1 = cv2.imread(image1_path, cv2.IMREAD_GRAYSCALE)
image2 = cv2.imread(image2_path, cv2.IMREAD_GRAYSCALE)

# Kontrast für Anzeige erhöhen
image1_display = cv2.normalize(image1, None, 0, 255, cv2.NORM_MINMAX)
image2_display = cv2.normalize(image2, None, 0, 255, cv2.NORM_MINMAX)

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        img, name = param
        value = img[y, x]
        print(f"{name}: Klick bei ({x}, {y}) - Pixelwert: {value}")

cv2.namedWindow('Left Image')
cv2.setMouseCallback('Left Image', mouse_callback, param=(image1, 'Left Image'))
cv2.namedWindow('Right Image')
cv2.setMouseCallback('Right Image', mouse_callback, param=(image2, 'Right Image'))

cv2.imshow('Left Image', image1_display)
cv2.imshow('Right Image', image2_display)
cv2.waitKey(0)
cv2.destroyAllWindows()