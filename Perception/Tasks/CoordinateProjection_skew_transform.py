import numpy as np
import matplotlib.pyplot as plt

def world_to_camera_coordinates(p):
    """
    Converts world coordinates to camera coordinates.

    Args:
        p (numpy.ndarray): 3D point in the world coordinate system.

    Returns:
        numpy.ndarray: 3D point in the camera coordinate system.
    """
    return np.array([-p[1], -p[2], p[0]])

def project_points(p, K):
    """
    Projects a 3D point in the camera coordinate system to the image sensor coordinate system.

    Args:
        p_cam (numpy.ndarray): 3D point in the camera coordinate system.
        K (numpy.ndarray): Intrinsic camera matrix.

    Returns:
        numpy.ndarray: 2D image coordinates of the projected points.
    """
    # Perform the projection
    p_projected = np.dot(K, p)

    # Divide by the homogeneous coordinate
    u = int(p_projected[0, 0] / p_projected[2, 0])  # Extract scalar value
    v = int(p_projected[1, 0] / p_projected[2, 0])  # Extract scalar value

    return [u, v]

def verify_projection(image_path, p_img):
    """
    Verifies the projection by plotting the calculated center coordinates on the image.

    Args:
        image_path (str): Path to the image file.
        u (float): u-coordinate of the projected point.
        v (float): v-coordinate of the projected point.
    """
    img = plt.imread(image_path)
    plt.imshow(img)
    plt.scatter(p_img[0], p_img[1], color='red', label='Projected Point')
    plt.legend()
    plt.title("Projection Verification")
    plt.show()

def main():
    # Example focal length
    f = 50

    # Example 3D point in the world coordinate system
    p_w = np.array([[38.933], 
                  [-0.294], 
                  [0.583],
                  [1]])  # Homogeneous coordinates
        
    K = np.array([[1606.6,    0,   672.3,], 
                  [0,      1600.3, 331.5],
                  [0,         0,     1.0]])
    
    t_wc = np.array([[1.77],
                  [0.15],
                  [-0.6],
                  [1]])  # Translation vector from world to camera coordinates
    
    R_wc = np.array([[0.99927809,  0, 0.03799086],
                  [0,              1, 0],
                  [-0.03799086,    0, 0.99927809],
                  [0,              0, 0]])  # Rotation matrix from world to camera coordinates
    
    Rt_wc = np.hstack((R_wc, t_wc))  # Combine rotation and translation into a single matrix
    p_c = np.dot(Rt_wc, p_w)  # Transform the point to camera coordinates
    p_c = world_to_camera_coordinates(p_c)


    p_img = project_points(p_c, K)
    print("Projected Point:\n", p_img)

    image_path = f"C:\\Users\\lukas\\Dokumente\\Studium\\Master\\2. Semester\\Fächer\\Autonomous Systems Perception and Situation Understanding\\ASPSU-Projects\\Perception\\Tasks\\Images\\test_image.png"
    verify_projection(image_path, p_img)

if __name__ == "__main__":
    main()