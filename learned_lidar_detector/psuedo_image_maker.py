'''
This script converts the A9 data into BEV psuedo images
based on the config file values

Based on: https://github.com/maudzung/SFA3D
'''

import argparse
import os
from glob import glob
import json
from pypcd4 import PointCloud

import configs.a9_config as cfg

# create a class that takes the arguments and performs the tasks
class LidarBevCreator:
    def __init__(self, input_path, output_path):
        # store the input and output folder paths as attributes
        input_path = input_path
        self.output_path = output_path

        self.sensor_direction = 's110_lidar_ouster_north'

        self.lidar_folder = os.path.join(input_path, 'point_clouds', self.sensor_direction)
        self.gt_folder = os.path.join(input_path,'labels_point_clouds', self.sensor_direction)
        self.image_folder = os.path.join(input_path,'images', self.sensor_direction)

        print('Getting files from path.')
        self.lidar_list = self.get_files(self.lidar_folder, 'pcd')

    @staticmethod
    def get_files(path, ext):
        assert os.path.isdir(path)
        return glob(os.path.join(path, f'*.{ext}'))
    
    def pc_to_image(self):
        
        for pc_path in self.lidar_list:

            # Get detection bboxes in the ground plane
            gt_json = self.get_gt(pc_path)
            det_list = self.convert_a9_json(gt_json)

            # get point cloud as np.array
            pc = self.get_pc(pc_path)

            # Normalize pointcloud, align road plane with x-y plane

            # Rotate point cloud?

            # Crop plane based on paramters

            # Covvert to BEV

    def create_bev(self, pc):
        '''
        create 3 channel image
        1) density
        2) height map
        3) ? since a9 has no intensity data
        '''
        pass

    def get_gt(self, lidar_file):
        head, tail = os.path.split(lidar_file)
        name, _ = os.path.splitext(tail)
        gt_path = os.path.join(self.gt_folder, name + '.json')

        with open(gt_path) as f: 
            return json.load(f) 

    def convert_a9_json(self, gt_json):
        '''
        Covnert the A9 .json gt format into lists of yolo-obb format
        list = [[class, x1, y1, x2, y2, x3, y3, x4, y4]]
        '''
        frames = gt_json['openlabel']['frames']

        det_list = []

        # Only one frame per file
        frame_num = list(frames.keys())[0]
        objects = frames[frame_num]['objects']
        for item in objects.items():
            data = item[1]['object_data']
            bbox = data['cuboid']['val']
            
            '''
            @TODO add the corners to the detection list
            '''
            corners = self.bbox3d_to_corners(bbox)

            detection = [data['type'], ]
            det_list.append(detection)

        return det_list
    
    @staticmethod
    def bbox3d_to_corners(bbox):
        '''
        Take a9 label cuboid format to corners
        '''
        x, y, z = bbox[0], bbox[1], bbox[2]
        qx, qy, qz, qw = bbox[3], bbox[4], bbox[5], bbox[6]
        w, l, h = bbox[7], bbox[8], bbox[9]

        '''
        @TODO finish this function
        '''
        corners = None
        return corners
        

    def draw_r_bbox(self, corners, image):
        '''
        Draw rotted bbox on the psuedo image
        '''
        pass

        # use corner points to draw
    
    def get_pc(self, lidar_file):
        pc = PointCloud.from_path(lidar_file)
        return pc.numpy()



def main(args):
    lbc = LidarBevCreator(input_path=args.input, output_path=args.output)

    # Process the point clouds into images
    lbc.pc_to_image()


# check if the script is run directly and call the main function
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Processes the LiDAR images into BEV psuedo images in the YOLO-obb format.")
    parser.add_argument(
        "-i", "--input", help="The path of the A9 sequence.", type=str,
        default='/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s01')
    parser.add_argument(
        "-o", "--output", help="The path where the results are saved.", default=None)
    args = parser.parse_args()
    main(args)
