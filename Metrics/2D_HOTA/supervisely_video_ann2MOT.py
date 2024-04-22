'''
Convert superviesly json format to MOT csv.

Checks for duplicate object detections within the same frame (labelling error).
'''

import argparse
import csv
import json
import os
import random

def get_unique_id(key):
    seed_int = hash(key)
    random.seed(seed_int)
    random_uint32 = random.getrandbits(32)

    return random_uint32

def convert_json_to_csv(json_file):
    no_output_flag = False

    # Open the json file and load the data
    with open(json_file, "r") as f:
        data = json.load(f)

    id_dict = {}

    path, _ = os.path.splitext(json_file)
    output_filepath = path + '.txt'
    with open(output_filepath, "w") as f:
        writer = csv.writer(f)

        # Loop through the frames in the data
        for frame in data["frames"]:
            # Get the frame index
            frame_index = frame["index"]

            # Create a set to store the keys of the figures in the frame
            seen_keys = set()

            # Loop through the figures in the frame
            for figure in frame["figures"]:

                figure_key = figure["objectKey"]

                # Check if the figure key is already in the set
                if figure_key not in id_dict:
                    id_dict[figure_key] = (len(id_dict) + 1, frame_index)

                if figure_key not in seen_keys:
                    seen_keys.add(figure_key)
                else:
                    print(f'Error: multiple labels for the same object within frame {frame_index}')
                    no_output_flag = True

                id = id_dict[figure_key][0]

                geometry_points = figure["geometry"]["points"]
                x1, y1 = geometry_points['exterior'][0]
                x2, y2 = geometry_points['exterior'][1]

                width = x2 - x1
                height = y2 - y1

                if not no_output_flag:
                    writer.writerow([frame_index, id, x1, y1, width, height, -1, -1, -1, -1])


def main():
    # Create an argument parser
    parser = argparse.ArgumentParser(
        description="Convert superviselyjson to MOT csv.")
    parser.add_argument("-i", "--input", help="The path to the json file.", type=str,
                        default='/home/adrian/Downloads/may10_q7_video/ds0/ann/output.mp4.json')
    args = parser.parse_args()

    # Call the convert function with the json file path
    convert_json_to_csv(args.input)


if __name__ == "__main__":
    main()
