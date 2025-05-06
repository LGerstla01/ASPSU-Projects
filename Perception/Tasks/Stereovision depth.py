# Stereovision depth calculation
# This code calculates the depth of an object using stereo vision principles.

pixel_size = 1.4173 * 1e-6
focal_length = 3 * 1e-3 
base_line = 0.3

pixel_pos_left = 446
pixel_pos_right = 435

z = focal_length * base_line / ((pixel_pos_left - pixel_pos_right)* pixel_size) 

print("depth:", z)