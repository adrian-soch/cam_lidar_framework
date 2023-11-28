'''
This script removes MOT BBoxes that are not inside the polygopn determiend by the provides points

The csv file is in the MOT format
    frame_num, id, bb_left, bb_top, bb_width, bb_height, conf, -1, -1, -1
    
Images x,y coords are indexed from 0, starting at the top left.
'''

import argparse
import cv2
import numpy as np
import os
from shapely.geometry import Point, Polygon
import pandas as pd

from re_number_frames import normalize_frame_numbers, get_lines, write_lines


def main(args):
    path, name = os.path.split(args.file_path)

    if args.output is None:
        result_name = os.path.join(path, 'processed_' + name)
    else:
        result_name = os.path.join(path, args.output)

    '''
    Start the frame numbers from 1, if not already the case

    This has to occur before the pruning detections
    '''
    lines = get_lines(args.file_path,)
    normalized_lines = normalize_frame_numbers(lines)
    write_lines(result_name, normalized_lines)
    
    # Define a list of points as tuples, must be clockwise

    # may10 short_range
    # roi_points = np.array([(658,305), (980,960), (1900,737), (1049,287)])

    # oct18
    roi_points = np.array([(5,735),(1270,645),(1500,676),(240,1060)])

    # dec7
    # roi_points = np.array([(835,480), (1050,1056), (1918,845), (1130,465)])

    # Create a polygon object from the points using shapely
    region_of_interest = Polygon(roi_points)
    if not region_of_interest.is_valid:
        print('Error: polygon not valid, check order of the points.')
        exit(-1)

    if args.image_path is not None:
        # Draw roi on image
        img = cv2.imread(args.image_path)

    # Read the csv file using pandas
    df = pd.read_csv(result_name, header=None)

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

        if args.image_path is not None:
            cv2.polylines(img, [points], True, colour, 2)

    if args.image_path is not None:
        # Draw roi on image
        cv2.polylines(img, [roi_points], True, (255, 0, 0), 3)

    # Drop the rows that are in the remove list using pandas
    df = df.drop(remove_indices)

    df.to_csv(result_name, index=False, header=None)

    if args.image_path is not None:
        # Show the image with polygons
        cv2.imshow("Image with polygons", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Description of your program')
    parser.add_argument('--file_path', '-f', type=str, required=True,
                        help='Path to file with detections in MOT format.')
    parser.add_argument('--image_path', '-i', type=str, default=None,
                        help='(Optional) Path to image for visualizing detections in a sequence.')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='(Optional) Path to image for visualizing detections in a sequence.')
    args = parser.parse_args()
    main(args)
