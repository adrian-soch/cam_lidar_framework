import cv2
import numpy as np
import matplotlib.pyplot as plt

# Read the image from the given source
IMG_PATH = '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s01/images/s110_camera_basler_south2_8mm/1646667310_055996268_s110_camera_basler_south2_8mm.jpg'
img = cv2.imread(IMG_PATH)

# Define the source and destination points
# src = np.array([[1075,440], [1193,464], [968,490], [1098, 522]])
src = np.array([[1384,458], [1706,482], [962,760], [1430, 1054]])
dst = np.array([[1000, 400], [1300, 400], [1000, 800], [1300, 800]])

# Find the homography matrix
H, _ = cv2.findHomography(src, dst)

# Apply the homography transformation to the image
img_warped = cv2.warpPerspective(img, H, (img.shape[1], img.shape[0]))

for points in src:
    img = cv2.circle(img, points, 9, (255, 0, 0), -1)

for points in dst:
    img_warped = cv2.circle(img_warped, points, 9, (0, 255, 0), -1)

img_concat = np.hstack((img, img_warped))

plt.imshow(img)
plt.imshow(img_concat)
plt.show()


