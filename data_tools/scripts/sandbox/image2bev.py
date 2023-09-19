import cv2
import numpy as np
import matplotlib.pyplot as plt

# Read the image from the given source
IMG_PATH = '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s01/images/s110_camera_basler_south2_8mm/1646667310_055996268_s110_camera_basler_south2_8mm.jpg'
img = cv2.imread(IMG_PATH)

# Define the source and destination points
src = np.array([[1075,440], [1193,464], [968,490], [1098, 522]])
dst = np.array([[500, 500], [650, 500], [500, 650], [650, 650]])

# Find the homography matrix
H, _ = cv2.findHomography(src, dst)

'''
Let K be the intrinsic matrix of the camera, R be the rotation matrix and t be the translation vector that describe the transform between the camera and the ground plane.
The homography matrix H is given by H = K * (R - t * n^T) / d, where n is the normal vector of the ground plane and d is the distance from the camera to the ground plane. You can find more details about this formula in this answer.
'''
# # Define the intrinsic matrix K of the camera
# K = np.array([[fx, 0, cx],
#               [0, fy, cy],
#               [0, 0, 1]])

# # Define the rotation matrix R and the translation vector t that describe the transform between the camera and the ground plane
# R = np.array([[r11, r12, r13],
#               [r21, r22, r23],
#               [r31, r32, r33]])
# t = np.array([tx, ty, tz])

# # Define the normal vector n and the distance d of the ground plane
# # If you have the equation of the plane in the form Ax + By + Cz + D = 0,
# # then the normal vector is given by (A, B, C)

# n = np.array([nx, ny, nz])
# d = dz

# # Compute the homography matrix H using the formula H = K * (R - t * n^T) / d
# Hcalc = K @ (R - t[:, np.newaxis] @ n[np.newaxis, :]) / d

# Print the homography matrix H
print(H)

# Apply the homography transformation to the image
img_warped = cv2.warpPerspective(img, H, (img.shape[1], img.shape[0]))

img_concat = np.hstack((img, img_warped))

# # Show the concatenated image using imshow
# cv2.imshow("Original and Warped", img_concat)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
plt.imshow(img)
plt.imshow(img_concat)
plt.show()


