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

def project_points(p, M):
    """
    Projects a 3D point in the world coordinate system to the image sensor coordinate system.

    Args:
        p (numpy.ndarray): 3D point in the world coordinate system (homogeneous coordinates).
        M (numpy.ndarray): Full camera projection matrix.

    Returns:
        numpy.ndarray: 2D image coordinates of the projected points.
    """
    # Perform the projection
    p_projected = np.dot(M, p)

    # Divide by the homogeneous coordinate
    u = p_projected[0] / p_projected[2]
    v = p_projected[1] / p_projected[2]

    return [int(u), int(v)]

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
    p = np.array([[38.933], [-0.294], [0.583]])  # Homogeneous coordinates
    p = world_to_camera_coordinates(p)
    p = np.append(p, 1)  # Add homogeneous coordinate

    K = np.array([[1606.6,    0,   672.3,], 
                  [0,      1600.3, 331.5],
                  [0,         0,     1.0],
                  [0,         0,     0]])
    
    t = np.array([[1.77],
                  [0.15],
                  [-0.6],
                  [1]])  # Homogeneous coordinates
    
    t = world_to_camera_coordinates(t)

    R = np.array([[0.99927809,  0, 0.03799086],
                  [0,           1, 0],
                  [-0.03799086, 0, 0.99927809]])
    
    # Combine R and t into a 3x4 matrix
    Rt = np.hstack((R, t))

    # Compute the full camera projection matrix
    M = np.matmul(K, Rt)

    # Project points using the full projection matrix
    p_img = project_points(p, M)
    print("Projected Points:\n", p_img)

    # Verify projection (replace 'image_path' with the actual path to your image)
    image_path = f"C:\\Users\\lukas\\Dokumente\\Studium\\Master\\2. Semester\\Fächer\\Autonomous Systems Perception and Situation Understanding\\ASPSU-Projects\\Perception\\Tasks\\Images\\test_image.png"
    verify_projection(image_path, p_img)

if __name__ == "__main__":
    main()