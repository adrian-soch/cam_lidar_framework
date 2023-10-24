'''
This script removes MOT BBoxes that are not inside the polygopn determiend by the provides points

The csv file is in the MOT format
    frame_num, id, bb_left, bb_top, bb_width, bb_height, conf, -1, -1, -1
    
Images x,y coords are indexed from 0, starting at the top left.
'''

import cv2
import numpy as np
import os
from shapely.geometry import Point, Polygon
import pandas as pd

# Comment out to control image preview
SHOW_IMAGE = True
image_path = '/home/adrian/dev/metrics/test_may10000000.jpg'
file_path = '/home/adrian/dev/metrics/SORT_Results/fusion_yolov5l_strongsort+lidar.txt'

def main():
    # Define a list of points as tuples, must be clockwise
    # q7_2_may10_2023 bag
    roi_points = np.array([(495, 230), (900, 230), (1910, 730), (525, 900)])

    # Create a polygon object from the points using shapely
    region_of_interest = Polygon(roi_points)
    if not region_of_interest.is_valid:
        print('Error: polygon not valid, check order of the points.')
        exit(-1)

    if SHOW_IMAGE:
        # Draw roi on image
        img = cv2.imread(image_path)
        cv2.polylines(img, [roi_points], True, (255, 0, 0), 3)

    # Read the csv file using pandas
    df = pd.read_csv(file_path, header=None)

    # Create an empty list to store the indices of the rows to be removed
    remove_indices = []
    for index in range(len(df)):

        x_left = df.iloc[index, 2]
        x_right = x_left + df.iloc[index, 4]
        y_top = df.iloc[index, 3]
        y_bottom = y_top + df.iloc[index, 5]

        points = np.array([(x_left, y_top), (x_right, y_top),
                        (x_right, y_bottom), (x_left, y_bottom)], dtype=np.int32)

        lower_left = Point((x_left, y_bottom))
        lower_right = Point((x_right, y_bottom))
        colour = (0, 255, 0)

        # Check if the point is inside the polygon using shapely
        if not (region_of_interest.contains(lower_left) and region_of_interest.contains(lower_right)):
            remove_indices.append(index)
            colour = (0, 0, 255)

        if SHOW_IMAGE:
            cv2.polylines(img, [points], True, colour, 2)

    # Drop the rows that are in the remove list using pandas
    df = df.drop(remove_indices)

    if SHOW_IMAGE:
        # Show the image with polygons
        cv2.imshow("Image with polygons", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    path, name = os.path.split(file_path)
    result = os.path.join(path, 'processed_' + name)
    df.to_csv(result, index=False, header=None)


if __name__ == "__main__":
    main()