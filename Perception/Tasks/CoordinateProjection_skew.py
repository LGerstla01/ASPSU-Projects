import numpy as np
import matplotlib.pyplot as plt

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
    u = int(p_projected[0] / p_projected[2])
    v = int(p_projected[1] / p_projected[2])

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

    # Example 3D points
    p = 40.673, -0.144, -1.563

    K = np.array([[1606.6,    0,   672.3],
                  [0,      1600.3, 331.5],
                  [0,         0,     1.0]])
    
    p_cam = [-p[1], -p[2], p[0]]


    # Project points
    p_img = project_points(p_cam, K)
    print("Projected Points:\n", p_img)

    # Verify projection (replace 'image_path' with the actual path to your image)
    image_path = f"C:\\Users\\lukas\\Dokumente\\Studium\\Master\\2. Semester\\Fächer\\Autonomous Systems Perception and Situation Understanding\\ASPSU-Projects\\Perception\\Tasks\\Images\\test_image.png"
    verify_projection(image_path, p_img)

if __name__ == "__main__":
    main()