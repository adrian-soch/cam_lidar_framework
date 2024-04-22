import json
import os
import numpy as np
from shapely.geometry import Point, Polygon

isGT = True

# Define the file path for the COCO .json file
file_path = "/home/adrian/dev/metrics/COCO_DATA/GT_COCO/oct18_r9_COCO.json"

# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
# Define a list of points as tuples, must be clockwise

# may10 short_range
# roi_points = np.array([(658, 305), (980, 960), (1900, 737), (1049, 287)])

# oct18
roi_points = np.array([(5,735),(1270,645),(1500,676),(240,1060)])

# dec7_2022
# roi_points = np.array([(835,480), (1050,1056), (1918,845), (1130,465)])

# dec14_2023
# roi_points = np.array([(20,720),(1910,755),(1910,575),(1825,605),(20,630)])

# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

# Load the json file as a dictionary
with open(file_path, "r") as f:
    coco_data = json.load(f)

if isGT:
    # Get the list of annotations from the dictionary
    annotations = coco_data["annotations"]
else:
    annotations = coco_data

# Create a polygon object from the points using shapely
region_of_interest = Polygon(roi_points)
if not region_of_interest.is_valid:
    print('Error: polygon not valid, check order of the points.')
    exit(-1)

idx_to_remove = []
for idx, ann in enumerate(annotations):
    # Get the bounding box coordinates from the annotation
    bbox = ann["bbox"] # x1, y1, w, h
    x_left, x_right = bbox[0], bbox[0] + bbox[2]
    y_bottom = bbox[1] + bbox[3]
    lower_left = Point((x_left, y_bottom))
    lower_right = Point((x_right, y_bottom))

    # Check if the point is inside the polygon using shapely
    if not (region_of_interest.contains(lower_left) and region_of_interest.contains(lower_right)):
        idx_to_remove.append(idx)


# Remove the detections outside the RoI

for index in sorted(idx_to_remove, reverse=True):
    del annotations[index]

if isGT:
    coco_data["annotations"] = annotations
else:
    coco_data = annotations

name, ext = os.path.splitext(file_path)
out_file = name + '_Filtered' + ext
with open(out_file, 'w', encoding='utf-8') as f:
    json.dump(coco_data, f, ensure_ascii=True, indent=4)
