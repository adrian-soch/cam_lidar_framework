# Import numpy for array operations
import numpy as np

# Define a function to calculate the IoU between two 3D bounding boxes
def iou_3d(box1, box2):
    # Extract the coordinates and dimensions of each box
    x1, y1, z1, l1, w1, h1, yaw1 = box1
    x2, y2, z2, l2, w2, h2, yaw2 = box2

    # Calculate the vertices of each box
    v1 = np.array([[x1 + l1 / 2, y1 + w1 / 2, z1 + h1 / 2],
                    [x1 + l1 / 2, y1 + w1 / 2, z1 - h1 / 2],
                    [x1 + l1 / 2, y1 - w1 / 2, z1 + h1 / 2],
                    [x1 + l1 / 2, y1 - w1 / 2, z1 - h1 / 2],
                    [x1 - l1 / 2, y1 + w1 / 2, z1 + h1 / 2],
                    [x1 - l1 / 2, y1 + w1 / 2, z1 - h1 / 2],
                    [x1 - l1 / 2, y1 - w1 / 2, z1 + h1 / 2],
                    [x1 - l1 / 2, y1 - w1 / 2, z1 - h1 / 2]])

    v2 = np.array([[x2 + l2 / 2, y2 + w2 / 2, z2 + h2 / 2],
                    [x2 + l2 / 2, y2 + w2 / 2, z2 - h2 / 2],
                    [x2 + l2 / 2, y2 - w2 / 2, z2 + h2 / 2],
                    [x2 + l2 / 2, y2 - w2 / 2, z2 - h2 / 2],
                    [x2 - l2 / 2, y2 + w2 / 2, z2 + h2 / 2],
                    [x2 - l2 / 2, y2 + w2 / 2, z2 - h2 / 2],
                    [x2 - l2 / 2, y2 - w2 / 2, z2 + h2 / 2],
                    [x2 - l2 / 2, y2 - w2 / 2, z2 - h2 / 2]])

    # Rotate the vertices by the yaw angle
    v1 = np.dot(v1 - np.array([x1, y1, z1]), np.array([[np.cos(yaw1), -np.sin(yaw1), 0],
                                                        [np.sin(yaw1), np.cos(yaw1), 0],
                                                        [0, 0, 1]])) + np.array([x1, y1, z1])

    v2 = np.dot(v2 - np.array([x2, y2, z2]), np.array([[np.cos(yaw2), -np.sin(yaw2), 0],
                                                        [np.sin(yaw2), np.cos(yaw2), 0],
                                                        [0, 0, 1]])) + np.array([x2, y2, z2])

    # Find the min and max coordinates of each box along each axis
    x_min1, y_min1, z_min1 = np.min(v1, axis=0)
    x_max1, y_max1, z_max1 = np.max(v1, axis=0)
    x_min2, y_min2, z_min2 = np.min(v2, axis=0)
    x_max2, y_max2, z_max2 = np.max(v2, axis=0)

    # Calculate the intersection volume
    x_inter = max(0, min(x_max1, x_max2) - max(x_min1, x_min2))
    y_inter = max(0, min(y_max1, y_max2) - max(y_min1, y_min2))
    z_inter = max(0, min(z_max1, z_max2) - max(z_min1, z_min2))
    inter_vol = x_inter * y_inter * z_inter

    # Calculate the union volume
    vol1 = l1 * w1 * h1
    vol2 = l2 * w2 * h2
    union_vol = vol1 + vol2 - inter_vol

    # Calculate the IoU
    iou = inter_vol / union_vol

    return iou

# Define a function to calculate the precision, recall, and F1 score for a given IoU threshold
def prf_3d(gt, det, iou_th):
    # Initialize the true positive, false positive, and false negative counts
    tp = 0
    fp = 0
    fn = 0

  # Loop through the ground truth boxes
    for box1 in gt:
    # Initialize a flag to indicate if the box is matched
        matched = False
    # Loop through the detection boxes
        for box2 in det:
        # Calculate the IoU between the boxes
            iou = iou_3d(box1, box2)
            # If the IoU is above the threshold, mark the boxes as matched and increment the true positive count
            if iou >= iou_th:
                    matched = True
                    tp += 1
                    break
        # If the box is not matched, increment the false negative count
        if not matched:
            fn += 1

    # Calculate the false positive count as the difference between the number of detections and true positives
    fp = len(det) - tp

    # Calculate the precision, recall, and F1 score
    precision = tp / (tp + fp) if tp + fp > 0 else 0
    recall = tp / (tp + fn) if tp + fn > 0 else 0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall > 0 else 0

    return precision, recall, f1

# Define a list of ground truth boxes
gt = [[1, 2, 3, 4, 5, 6, 0.1],
      [2, 3, 4, 5, 6, 7, 0.2],
      [3, 4, 5, 6, 7, 8, 0.3]]

# Define a list of detection boxes
det = [[1.1, 2.1, 3.1, 4.1, 5.1, 6.1, 0.1],
       [2.1, 3.1, 4.1, 5.1, 6.1, 7.1, 0.2],
       [4, 5, 6, 7, 8, 9, 0.4]]

# Define an IoU threshold
iou_th = 0.5

# Calculate the precision, recall, and F1 score
precision, recall, f1 = prf_3d(gt, det, iou_th)

# Print the results
print(f"Precision: {precision:.3f}")
print(f"Recall: {recall:.3f}")
print(f"F1 score: {f1:.3f}")
