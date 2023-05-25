import os
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("-f", "--file", help="name of the file", required=True)
args = parser.parse_args()

# Get the name of the file from the arguments
file_name = args.file

# Open the file in read mode
with open(file_name, "r") as f:
    # Read the file as a list of lines
    lines = f.readlines()

# Shift frame numbers to start at 1
shift = int(lines[0].split(",")[0])

if(shift > 1):
    shift -= 1
else:
    shift += 1

# Loop through the list of lines with an index
for i, line in enumerate(lines):
    # Split the line by the comma character
    values = line.split(",")

    values[0] = str(int(values[0]) - shift)

    # Join the values back by the comma character
    new_line = ",".join(values)
    lines[i] = new_line

# Open the file in write mode
path, file = os.path.split(file_name)

file = 're-numbered-' + file
file_name = os.path.join(path, file)
with open(file_name, "w") as f:
    # Write the modified list of lines back to the file
    f.writelines(lines)


    # Initialize a counter for the increasing integers
counter = 1

# Initialize a set to store the seen values
seen = set()