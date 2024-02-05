"""
This script computes coco style mAP.
Requires .json files in COCO GT format, and COCO Detector format
"""

from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval
import numpy as np

# [GT File, Det File]
files = [['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/dec7_dhd1_short.mp4_COCO_Filtered.json','/home/adrian/dev/metrics/COCO_DATA/detector_COCO/Filtered/dec7_dhdh1_short_COCO_Filtered.json'],
          ['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/dec14_ok_COCO_Filtered.json','/home/adrian/dev/metrics/COCO_DATA/detector_COCO/dec14_2023_ok_COCO.json'],
           ['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/may10_q7_COCO_Filtered.json','/home/adrian/dev/metrics/COCO_DATA/detector_COCO/Filtered/may10_q7_COCO_Filtered.json'],
           ['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/may10_q5_COCO_Filtered.json','/home/adrian/dev/metrics/COCO_DATA/detector_COCO/Filtered/may10_r5_COCO_Filtered.json'],
           ['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/oct18_r1_COCO_Filtered.json','/home/adrian/dev/metrics/COCO_DATA/detector_COCO/Filtered/oct18_r1_COCO_Filtered.json'],
           ['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/oct18_r9_COCO_Filtered.json','/home/adrian/dev/metrics/COCO_DATA/detector_COCO/Filtered/oct18_r9_COCO_Filtered.json'],
           ]

# Initialize an empty list to store the mAP values for each pair
mAP_list = []

# Loop through the data array
for pair in files:
    # Load the GT and Det json files using COCO    
    coco_gt = COCO(pair[0]) # GT file
    coco_dt = coco_gt.loadRes(pair[1])

    # Create a COCO evaluator object
    coco_eval = COCOeval(cocoGt=coco_gt, cocoDt=coco_dt, iouType="bbox")

    # 0-Pedestrian, 2-Car
    coco_eval.params.catIds = [2]

    # Evaluate the detections
    coco_eval.evaluate()
    coco_eval.accumulate()
    coco_eval.summarize()

    # Get the mAP value (the average of the AP values for each IoU threshold)
    mAP_05_095 = coco_eval.stats[0]
    mAP_05 = coco_eval.stats[1]

    # Append the mAP value to the list
    mAP_list.append([mAP_05_095, mAP_05])

# Print the mAP list
print(np.mean(mAP_list, axis=0))
print(mAP_list)