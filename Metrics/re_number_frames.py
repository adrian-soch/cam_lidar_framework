import os
import argparse

def get_lines(file_name):
    with open(file_name, "r") as f:
        # Read the file as a list of lines
        lines = f.readlines()
    return lines

def write_lines(file_name, lines):
    with open(file_name, "w") as f:
        # Write the modified list of lines back to the file
        f.writelines(lines)

def normalize_frame_numbers(lines):
    # Shift frame numbers to start at 1
    shift = int(lines[0].split(",")[0])

    if shift == 1:
        return lines
    else:
        shift -= 1

    # Loop through the list of lines with an index
    for i, line in enumerate(lines):
        # Split the line by the comma character
        values = line.split(",")

        values[0] = str(int(values[0]) - shift)

        # Join the values back by the comma character
        new_line = ",".join(values)
        lines[i] = new_line

    return lines

def main(args):
    # Open the file in read mode
    file_name = args.file
    lines = get_lines(file_name)

    normalized_lines = normalize_frame_numbers(lines)

    path, file = os.path.split(file_name)
    file = 're-numbered-' + file
    file_name = os.path.join(path, file)

    write_lines(file_name, normalized_lines)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", help="name of the file", required=True)
    args = parser.parse_args()

    # Get the name of the file from the arguments
    main(args)
