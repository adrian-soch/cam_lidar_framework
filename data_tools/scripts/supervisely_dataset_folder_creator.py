""" Helper script to create a folder struture like this for uploaing image context to supervisely tool:

    my_project
└── dataset_01
    ├── frame.pcd
    ├── kitti_0000000001.pcd
    └── related_images
        └── kitti_0000000001_pcd
            ├── kitti_0000000001.png
            └── kitti_0000000001.png.json

    Code was generated with chat GPT to speed up the process
"""

import os
import glob
import shutil
import argparse

def createRelatedImageFolders(root_folder, json_file):

    # Iterate through all the images in the root folder
    for filename in os.listdir(root_folder):
        # Create a folder with the same name as the file
        foldername, _ = os.path.splitext(filename)
        foldername = foldername + '_pcd'
        folderpath = os.path.join(root_folder, foldername)
        os.makedirs(folderpath, exist_ok=True)
        
        # Move the file into the corresponding folder
        filepath = os.path.join(root_folder, filename)
        shutil.move(filepath, os.path.join(folderpath, filename))

        # Copy the file into the folder
        shutil.copy(json_file, folderpath)

        # Rename json to match image
        _, json_name = os.path.split(json_file)
        _, image_name = os.path.split(glob.glob(folderpath + '/*.jpg')[0])
        new_json_name = image_name + '.json'
        os.rename(os.path.join(folderpath,json_name), os.path.join(folderpath,new_json_name))
            
        print(f"Moved {filename} to {foldername} folder and Copied and renamed json file.")

if __name__ == '__main__':

    # Define the argparse
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--root_folder', '-r', type=str, default='/home/adrian/dev/metrics/dec7_daytime_dataset/related_images',
            help='The path to the root folder of all the related images.')
    
    parser.add_argument('--json_file', '-j', type=str, default='/home/adrian/dev/metrics/000240.jpg.json',
            help='The path to the json file with transformations.')
    
    # Parse the arguments
    args = parser.parse_args()
    
    # Call the main function
    createRelatedImageFolders(args.root_folder, args.json_file)
