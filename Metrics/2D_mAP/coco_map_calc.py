"""
This script computes coco style mAP.
Requires .json files in COCO GT format, and COCO Detector format
"""

from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval
import numpy as np

# [GT File, Det File]
files = [
['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/may10_q5_COCO_Filtered.json', '/home/adrian/dev/metrics/COCO_DATA/lidar_COCO/filtered/may10_r5_2024-02-13T22:57:51_COCO_Filtered.json'],
['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/may10_q7_COCO_Filtered.json', '/home/adrian/dev/metrics/COCO_DATA/lidar_COCO/filtered/may10_q72024-02-13T22:54:52_COCO_Filtered.json'],
['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/dec7_dhd1_short.mp4_COCO_Filtered.json', '/home/adrian/dev/metrics/COCO_DATA/lidar_COCO/filtered/dec7_2024-02-13T22:40:04_COCO_Filtered.json'],
['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/dec14_ok_COCO_Filtered.json', '/home/adrian/dev/metrics/COCO_DATA/lidar_COCO/filtered/dec142024-02-13T23:05:56_COCO_Filtered.json'],
['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/oct18_r1_COCO_Filtered.json', '/home/adrian/dev/metrics/COCO_DATA/lidar_COCO/filtered/oct18_r12024-02-13T23:01:12_COCO_Filtered.json'],
['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/oct18_r9_COCO_Filtered.json', '/home/adrian/dev/metrics/COCO_DATA/lidar_COCO/filtered/oct18_r9_2024-02-13T22:51:59_COCO_Filtered.json'],
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
    coco_eval.params.catIds = [0]

    # Change IoU thresh is desired
    # coco_eval.params.iouThrs = [0.001]

    # Evaluate the detections
    coco_eval.evaluate()
    coco_eval.accumulate()
    coco_eval.summarize()

    '''
    https://github.com/cocodataset/cocoapi/issues/572

    
    For each image find the TP and FN, then add up

    #now I will use the dict produced by the evaluate() function to calculate all

    #here i simply select the dict that corresponds to 'aRng': [0, 10000000000.0]
    image_evaluation_dict = coco_eval.evalImgs[0]

    #select the index related to IoU = 0.5
    iou_treshold_index = 0

    #all the detections from the model, it is a numpy of True/False (In my case they are all False)
    detection_ignore = image_evaluation_dict["dtIgnore"][iou_treshold_index]

    #here we consider the detection that we can not ignore (we use the not operator on every element of the array)
    mask = ~detection_ignore

    #detections number
    n_ignored = detection_ignore.sum()

    #and finally we calculate tp, fp and the total positives
    tp = (image_evaluation_dict["dtMatches"][iou_treshold_index][mask] > 0).sum()
    fp = (image_evaluation_dict["dtMatches"][iou_treshold_index][mask] == 0).sum()
    n_gt = len(image_evaluation_dict["gtIds"]) - image_evaluation_dict["gtIgnore"].astype(int).sum()
    '''
    print(coco_eval.evalImgs[0])

    # # Get the mAP value (the average of the AP values for each IoU threshold)
    # mAP_05_095 = coco_eval.stats[0]
    # mAP_05 = coco_eval.stats[1]

    # # Append the mAP value to the list
    # mAP_list.append([mAP_05_095, mAP_05])

    # # dict_keys(['params', 'counts', 'date', 'precision', 'recall', 'scores'])
    # tp = coco_eval.computeOks
    # print(tp)
    # # fp = coco_eval.eval['fp'][0, 0]

# Print the mAP list
print(np.mean(mAP_list, axis=0))
print(mAP_list)