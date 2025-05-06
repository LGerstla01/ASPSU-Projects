import numpy as np

def project_points(p, f):
    """
    Projects 3D points to 2D image coordinates.

    Args:
        p (numpy.ndarray): 3D points in the world coordinate system.
        f (float): Camera focal length.

    Returns:
        numpy.ndarray: 2D image coordinates of the projected points.
    """
    p = np.append(p, 1)  # Homogeneous coordinate
    f_mat = np.array([[f, 0, 0, 0],
                      [0, f, 0, 0],
                      [0, 0, 1, 0]])

    p_proj = np.dot(f_mat, p)
    
    p_image = [int(p_proj[0] / p_proj[2]), int(p_proj[1] / p_proj[2])]

    return p_image

def main():
    # Example focal length
    f = 50

    # Example 3D points
    p = np.array([200, 200, 120])

    # Project points
    p_img = project_points(p, f)
    print("Projected Points:\n", p_img)

if __name__ == "__main__":
    main()