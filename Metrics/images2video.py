import argparse
import cv2
import glob
import os

parser = argparse.ArgumentParser(description="Create a video from images in a folder")
parser.add_argument("folder", type=str, help="The path to the folder containing the images")
args = parser.parse_args()
folder = args.folder

if not os.path.exists(folder) or not os.path.isdir(folder):
    print("Invalid folder path")
    exit()

# Get the list of image files in the folder
image_files = glob.glob(os.path.join(folder, "*.jpg"))

# Sort the image files by their numerical names
image_files.sort(key=lambda x: int(os.path.basename(x).split(".")[0]))

if not image_files:
    print("No image files found in the folder")
    exit()

# Read the first image file and get its dimensions
img = cv2.imread(image_files[0])
height, width, channels = img.shape

# Create a video writer object with fourcc codec, fps and frame size
fourcc = cv2.VideoWriter_fourcc(*"XVID")
fps = 10
output = os.path.join(folder, "output.avi")
video = cv2.VideoWriter(output, fourcc, fps, (width, height))

# Loop through the image files and write them to the video
for image_file in image_files:
    img = cv2.imread(image_file)
    video.write(img)

# Release the video writer object
video.release()

# Print a success message
print("Video created successfully")
