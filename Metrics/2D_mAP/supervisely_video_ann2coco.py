'''
Convert superviesly json format to COCO json format.

Checks for duplicate object detections within the same frame (labelling error).
'''

import argparse
from datetime import datetime as dt
import json
import os
import random

CLASS_MAP = {'pedestrian':0, 'bicycle':1, 'car':2, 'motorcycle':3, 'bus':5, 'truck':7}

class CocoEntry:
    """Contains the values and helper functions for the detections in COCO format
        See comment at the top of the file for more details.
    """
    def __init__(self, id:int, image_id:int, type:int, bb_left:int, bb_top:int,
                 bb_width:int, bb_height:int, image_width=1920, image_height=1080):
        self.id = id
        self.image_id = image_id
        self.type = type
        self.bb_left = bb_left
        self.bb_top = bb_top
        self.bb_width = bb_width
        self.bb_height = bb_height
        self.area = bb_width * bb_height
        self.iscrowd = 0
        self.image_width = image_width
        self.image_height = image_height

    def image_dict(self):
       return {
            "id": self.image_id,
            "file_name": str(self.image_id) + ".jpg",
            "height": self.image_height,
            "width": self.image_width,
            "license": None,
            "coco_url": None
        }

    def annotation_dict(self):
        return {
            "id": self.id,
            "image_id": self.image_id,
            "category_id": self.type,
            "bbox": [
                self.bb_left,
                self.bb_top,
                self.bb_width,
                self.bb_height
            ],
            "area": self.area,
            "iscrowd": self.iscrowd
        }

def get_unique_id(key):
    seed_int = hash(key)
    random.seed(seed_int)
    random_uint32 = random.getrandbits(32)

    return random_uint32

def convert_json_to_coco(json_file):

    # Open the json file and load the data
    with open(json_file, "r") as f:
        data = json.load(f)

    id_dict = {}
    image_width = data["size"]["width"]
    image_height = data["size"]["height"]

    # Used for getting the object class
    class_lookup_list = data["objects"]

    # For storing all detections
    coco_list = []

    # Loop through the frames in the data
    for frame in data["frames"]:
        # Get the frame index
        frame_index = frame["index"]

        # Create a set to store the keys of the figures in the frame
        seen_keys = set()

        # Loop through the figures in the frame
        for figure in frame["figures"]:

            object_key = figure["objectKey"]
            
            # Check if the figure key is already in the set
            if object_key not in id_dict:
                id_dict[object_key] = (len(id_dict) + 1, frame_index)
                
            if object_key not in seen_keys:
                seen_keys.add(object_key)
            else:
                print(f'Error: multiple labels for the same object within frame {frame_index}')
                exit(-1)

            id = id_dict[object_key][0]
            geometry_points = figure["geometry"]["points"]
            x1, y1 = geometry_points['exterior'][0]
            x2, y2 = geometry_points['exterior'][1]
            width = x2 - x1
            height = y2 - y1

            type_str = get_type(class_lookup_list, object_key)
            if type_str.lower() in CLASS_MAP:
                type = CLASS_MAP[type_str.lower()]
            else:
                print(f'Error, no class: {type_str}')
                exit(-1)

            detection = CocoEntry(id=id, image_id=frame_index, type=type, bb_left=x1, bb_top=y1, bb_width=width,
                        bb_height=height, image_width=image_width, image_height=image_height)
            coco_list.append(detection)
            
    return coco_list

def write_to_json(output_filepath, coco_list):
    json_data = {
        "info": {
        "year": "2024",
        "version": "v0.0.1",
        "contributor": "Adrian",
        "url": None,
        "date_created": dt.now().strftime("%Y-%m-%dT%H:%M:%S")
    },
    "licenses": [],
    "categories": [
        {
            "id": 0,
            "name": "pedestrian",
            "supercategory": None
        },
        {
            "id": 1,
            "name": "bicycle",
            "supercategory": None
        },
        {
            "id": 2,
            "name": "car",
            "supercategory": None
        },
        {
            "id": 3,
            "name": "motorcycle",
            "supercategory": None
        },
        {
            "id": 5,
            "name": "bus",
            "supercategory": None
        },
        {
            "id": 7,
            "name": "truck",
            "supercategory": None
        }
    ],
    "images": [],
    "annotations": []
    }

    # Output the json in the COCO format
    # images first
    for data in coco_list:
        json_data["images"].append(data.image_dict())
        json_data["annotations"].append(data.annotation_dict())

    with open(output_filepath, "w", encoding='utf-8') as f:
        json.dump(json_data, f, ensure_ascii=False, indent=4)

def get_type(list, object_key):
    object_class = None
    for object in list:
        if object["key"] == object_key:
            object_class = object["classTitle"]

    return object_class


def main():
    # Create an argument parser
    parser = argparse.ArgumentParser(
        description="Convert superviselyjson to MOT csv.")
    parser.add_argument("-i", "--input", help="The path to the json file.", type=str,
                        default='/home/adrian/dev/metrics/ground_truth/dec14_2023_ok_clean/ds0/ann/output.mp4.json')
    args = parser.parse_args()

    # Call the convert function with the json file path
    coco_list = convert_json_to_coco(args.input)

    path, _ = os.path.splitext(args.input)
    out = path + '_COCO' + '.json'

    # Save the coco results into the json
    write_to_json(output_filepath=out, coco_list=coco_list)


if __name__ == "__main__":
    main()
