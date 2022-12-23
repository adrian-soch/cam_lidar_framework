#! /usr/bin/env python3
"""
@file projection_utils.py

@brief Contains utilities for projection and pointcloud manipulation

@section Author(s)
- Created by Adrian Sochaniwsky on 3/12/2022
"""

import yaml
import numpy as np

from .point_cloud2 import read_points

def get_calib_from_file(filepath, names, intrinsic=False, debug=False):
    """
    Parameters: filename of the calib file
                name of the calibration(s) as a list
    Return:
        List of calibrations in the order requested
            [R | t] shape(3*4)
    """
    if isinstance(names, str):
        names = [names]

    fp = open(filepath, 'r')
    data = yaml.load(fp, Loader=yaml.Loader)
    fp.close()

    cals = []
    for name in names:
        if intrinsic:
            row, col = data[name]['rows'], data[name]['cols']
            cals.append(np.float32(np.asarray(data[name]['data']).reshape(row,col)))
        else:
            R = np.asarray(data[name]['rotation']).reshape(3,3)
            T = np.asarray(data[name]['translation']).reshape(3,1)
            cals.append(np.float32(np.hstack([R, T])))

    return cals

def pc2_numpy(pc):

    cloud = read_points(pc, field_names=['x', 'y', 'z', 'intensity','t','reflectivity', 'ring','ambient','range'], skip_nans=False)

    return cloud