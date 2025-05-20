import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from scipy.ndimage import gaussian_filter

# Load images and resize for speed
left_img_path = 'Perception\Projekt\Additional_files\Stereo_images\ConstructionSiteLeft\image0110_c0.pgm'
right_img_path = 'Perception\Projekt\Additional_files\Stereo_images\ConstructionSiteRight\image0110_c1.pgm'
left_img = cv2.imread(left_img_path, cv2.IMREAD_GRAYSCALE)
right_img = cv2.imread(right_img_path, cv2.IMREAD_GRAYSCALE)

# Resize images for faster computation
scale = 0.5  # 50% size
left_img = cv2.resize(left_img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
right_img = cv2.resize(right_img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

# Parameters
window_size = 9  # Try 5, 7, 9, 11 for optimal results
half_w = window_size // 2
max_disparity = 32  # Reduce for speed (adjust if needed)

h, w = left_img.shape
disparity_map = np.zeros((h, w), dtype=np.float32)

# Pad images to handle borders
left_pad = cv2.copyMakeBorder(left_img, half_w, half_w, max_disparity+half_w, half_w, cv2.BORDER_REFLECT)
right_pad = cv2.copyMakeBorder(right_img, half_w, half_w, max_disparity+half_w, half_w, cv2.BORDER_REFLECT)

def ncc_vec(window1, window2):
    # Vectorized NCC for two windows
    mean1 = window1 - np.mean(window1)
    mean2 = window2 - np.mean(window2)
    numerator = np.sum(mean1 * mean2)
    denominator = np.sqrt(np.sum(mean1 ** 2) * np.sum(mean2 ** 2))
    if denominator == 0:
        return -1
    return numerator / denominator

# Compute disparity map (step size 2 for speed)
step = 2
for y in range(half_w, h + half_w, step):
    for x in range(max_disparity + half_w, w + half_w, step):
        left_window = left_pad[y - half_w:y + half_w + 1, x - half_w:x + half_w + 1]
        ncc_scores = []
        for d in range(max_disparity):
            x_right = x - d
            right_window = right_pad[y - half_w:y + half_w + 1, x_right - half_w:x_right + half_w + 1]
            ncc_scores.append(ncc_vec(left_window, right_window))
        best_disp = np.argmax(ncc_scores)
        disparity_map[y - half_w, x - max_disparity - half_w] = best_disp

# Optional: Interpolate missing values due to step size
disparity_map = gaussian_filter(disparity_map, sigma=1)

# Normalize disparity for visualization
disp_vis = np.copy(disparity_map)
disp_vis[disp_vis == 0] = 1e-3  # avoid log(0)

plt.figure(figsize=(10, 6))
plt.title("Disparity Map (NCC, jet colormap, log scale, fast)")
plt.imshow(disp_vis, cmap='jet', norm=LogNorm())
plt.colorbar(label='Disparity')
plt.axis('off')
plt.tight_layout()
plt.show()
