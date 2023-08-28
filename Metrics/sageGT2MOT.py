import os
import json
import argparse
from datetime import datetime


def main(output_folder, input):
    with open(input, "r") as f:
        # Load the json data as a dictionary
        data = json.load(f)

    # Format the date and time to create the folder name
    now = datetime.now()
    time = now.strftime("%Y-%m-%d_%H-%M-%S")
    output_path = os.path.join(output_folder, time)
    os.makedirs(output_path)

    file = os.path.join(output_path, 'MOT20-01.txt')
    key_dict = {}
    with open(file, "w") as f:

        for item in data["tracking-annotations"]:
            frame_number = item["frame-number"] + 1  # for 1 based indexing

            for annotation in item["annotations"]:
                # Get the object-name, center-x and center-y from the annotation
                id = annotation["object-id"]
                id = add_key(id, key_dict)

                center_x = annotation["center-x"]
                center_y = annotation["center-y"]
                center_z = annotation["center-z"]
                size_x = annotation["length"]
                size_y = annotation["width"]

                bb_left = center_y - size_y/2.0
                bb_top = center_x - size_x/2.0
                # Format the annotation data as a string separated by commas
                annotation_str = "{},{},{:.4f},{:.4f},{:.4f},{:.4f},{},{:.4f},{:.4f},{:.4f}".format(
                    frame_number, id, bb_left, bb_top, size_y, size_x, -1,
                    center_x, center_y, center_z)

                f.write(annotation_str + '\n')

    remove_trailing_newline(file)


def remove_trailing_newline(file):
    with open(file, "r") as f:
        # Read the file as a single string
        content = f.read()

    # Remove any trailing whitespace characters from the string
    content = content.rstrip()

    # Open the txt file in write mode
    with open(file, "w") as f:
        # Write the modified string back to the file
        f.write(content)


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
        key_dict[key] = len(key_dict) + 1  # id's are indexed starting at 1
        return key_dict[key]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    # Add an argument for the output folder path
    parser.add_argument("-o", "--output", help="output folder path",
                        default='/home/adrian/dev/metrics/amazon_sage_data')
    parser.add_argument("-i", "--input", help="Sage Maker GT .json filepath.",
                        default='/home/adrian/dev/metrics/amazon_sage_data/SeqLabel.json')
    args = parser.parse_args()
    output_folder = args.output
    input = args.input

    main(output_folder, input)
