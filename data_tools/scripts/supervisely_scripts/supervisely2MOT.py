"""
This script converts supervisorly file output to MOT format. Since the Lidar Pointcloud has 3D detections
we will use 2D bounding box from the (x,y) plane (ground plane projection) 
to leverage the preexisting scripts for calculating the metrics

Inputs: 1) Path to the folder that contains the .json files from the superviesrly export
        2) Name of the output file that contains all the dections/tracks in CSV format
            Note: the date and time will be appending to this name
            Note: the file will be saved in the same folder as the original json files

Outputs:
        1) The output file will contain the detections in MOT format for an entire sequenece. Below is an example
        from https://motchallenge.net/:

 #########################################################################################
 #########################################################################################  
 #########################################################################################   
   
NOTE Instead of camera coordinates and pixels, we are using lidar coordiantes and meters.

NOTE Make sure your algorithm is making the same bbox labelling assumptions as this script,
        else you results will be inaccurate!

    Width is length in Y (meters)
    Height is lenght in X (meters)

    BBox Top is X position (meters)
    BBox Left is Y position (meters)

#########################################################################################
#########################################################################################
#########################################################################################

<frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>
The conf value contains the detection confidence in the det.txt files. For the ground truth, it acts as a flag whether the entry
 is to be considered. A value of 0 means that this particular instance is ignored in the evaluation, while any other 
 value can be used to mark it as active. For submitted results, all lines in the .txt file are considered. The world 
 coordinates x,y,z are ignored for the 2D challenge and can be filled with -1. Similarly, the bounding boxes are ignored 
 for the 3D challenge. However, each line is still required to contain 10 values.

All frame numbers, target IDs and bounding boxes are 1-based. Here is an example:

Tracking with bounding boxes
(MOT15, MOT16, MOT17, MOT20)
  1, 3, 794.27, 247.59, 71.245, 174.88, -1, -1, -1, -1
  1, 6, 1648.1, 119.61, 66.504, 163.24, -1, -1, -1, -1
  1, 8, 875.49, 399.98, 95.303, 233.93, -1, -1, -1, -1
  ...

NOTE ChatGPT was used to generate most of this code to speed up developement
"""

import os
import re
import json
import argparse
from datetime import datetime

class Object:
    """Contains the unique name and geometry of the object
    """
    def __init__(self, key, classtitle, geometry):
        self.key = key
        self.classtitle = classtitle
        self.geometry = geometry


class Geometry:
    """Contains the geometry of the 3D bounding box
    """
    def __init__(self, geometryType, position, rotation, dimensions):
        self.geometryType = geometryType
        self.position = position
        self.rotation = rotation
        self.dimensions = dimensions

class MotEntry:
    """Contains the values and helper functions for the detections in MOT Challenge format
        See comment at the top of the file for more details.
    """
    def __init__(self, frame, id=None, bb_left=None, bb_top=None, bb_width=None, bb_height=None, conf=-1):
        self.frame = frame
        self.id = id
        self.bb_left = bb_left
        self.bb_top = bb_top
        self.bb_width = bb_width
        self.bb_height = bb_height
        self.conf = conf
        self.x = -1
        self.y = -1
        self.z = -1

    def toStr(self):
        return "{},{},{:.4f},{:.4f},{:.4f},{:.4f},{},{},{},{}".format(
            self.frame, self.id, self.bb_left, self.bb_top, self.bb_width, self.bb_height,
            self.conf, self.x, self.y, self.z)
    
    def geometry2bbox(self, geometry):

        self.bb_width = geometry.dimensions['y']
        self.bb_height = geometry.dimensions['x']

        # Use centroid instead of corner
        # Y value
        self.bb_left = geometry.position['y']
        # X value
        self.bb_top = geometry.position['x']

def getSortedFileList(folder_path, file_pattern):
    """Return all the files that match the regular expression pattern in sorted order

    Args:
        folder_path (str):
        file_pattern (re): regular expression to match files to

    Returns:
        list: sorted files names
    """
    # Get all files in the folder
    files = os.listdir(folder_path)

    # Filter out files that do not match the expected filename pattern
    files = [f for f in files if re.match(file_pattern, f)]

    # Sort files based on the increasing number found in the filename
    files = sorted(files, key=lambda f: int(f[:6]))

    return files

def add_key(key, key_dict) -> int:
    """Given a key, create a new entry if its previously unseen and return its ID.
    If the object is not new to the dictionary then provide the existing ID.

    Args:
        key (str): Unique id
        key_dict (dictionary):

    Returns:
        int: the id of the provided key
    """
    if key in key_dict:
        return key_dict[key]
    else:
        key_dict[key] = len(key_dict) + 1 # id's are indexed starting at 1
        return key_dict[key]


def main(folder_path, output_name, label_type):

    # Get the current date and time
    now = datetime.now()

    # Format the date and time to create the folder name
    time = now.strftime("%Y-%m-%d_%H-%M-%S")

    out_path = os.path.join(folder_path, time + output_name)

    file_pattern = r"\d{6}\.pcd\.json"
    files = getSortedFileList(folder_path, file_pattern=file_pattern)
    
    # Print the sorted list of files
    frame_count = 0 # frame id is indexed starting at 1
    for f in files:
        frame_count += 1
        filepath = os.path.join(folder_path, f)
        print(filepath)

        # Load the JSON file
        with open(filepath) as f:
            data = json.load(f)

        # Parse the objects and their geometries
        objects = {}
        for obj in data["objects"]:
            objects[obj["key"]] = Object(obj["key"], obj["classTitle"], None)
        for fig in data["figures"]:
            objects[fig["objectKey"]].geometry = Geometry(
                fig["geometryType"],
                fig["geometry"]["position"],
                fig["geometry"]["rotation"],
                fig["geometry"]["dimensions"]
            )

        object_dict = {}
        with open(out_path, "a") as f:
            for key, obj in objects.items():
                print(f"Object Key: {key}")
                print(f"Class Title: {obj.classtitle}")
                if obj.geometry is not None:
                    print("Geometry:")
                    print(f"\tType: {obj.geometry.geometryType}")
                    print(f"\tPosition: ({obj.geometry.position['x']}, {obj.geometry.position['y']}, {obj.geometry.position['z']})")
                    print(f"\tRotation: ({obj.geometry.rotation['x']}, {obj.geometry.rotation['y']}, {obj.geometry.rotation['z']})")
                    print(f"\tDimensions: ({obj.geometry.dimensions['x']}, {obj.geometry.dimensions['y']}, {obj.geometry.dimensions['z']})")

                    if label_type is 'MOT2D':
                        # Get unique object ID                        # Get unique object ID

                        entry = MotEntry(frame=frame_count)
                        entry.id = add_key(key, object_dict)

                        # Set the rest of the values
                        entry.geometry2bbox(obj.geometry)
                    
                        f.write(entry.toStr() + "\n")
                    elif label_type is 'KITTI3D':
                        raise NotImplementedError

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process a file")
    parser.add_argument("--path",'-p', help="Path to the folder with supervisorly files",
                        default='/home/adrian/dev/metrics/label')
    parser.add_argument("--output_name",'-o', help="Name of output file.",
                        default='custom_MOT_labels.txt')
    parser.add_argument("--label_type",'-l', help="Type of label output, ['MOT2D', 'KITTI3D'].",
                        default='MOT2D',
                        default='custom_labels.txt')
    args = parser.parse_args()
    main(args.path, args.output_name, args.type)