from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval
import numpy as np

NUM_GT_FILES = 1

'''
Fix format of the results:

https://github.com/cocodataset/cocoapi/issues/401

'''

# [GT File, Det File]
# files = [
#     # camera-only
#     ['dec7_dhd1.json', ],
#     ['dec14_ok.json', ],
#     ['may10_q5.json', ],
#     ['may10_q7.json', ],
#     ['oct18_r1.json', ],
#     ['oct18_r9.json', ],
#     # lidar-only
#     ['dec7_dhd1.json', ],
#     ['dec14_ok.json', ],
#     ['may10_q5.json', ],
#     ['may10_q7.json', ],
#     ['oct18_r1.json', ],
#     ['oct18_r9.json', ],
#     # camera-lidar
#     ['dec7_dhd1.json', ],
#     ['dec14_ok.json', ],
#     ['may10_q5.json', ],
#     ['may10_q7.json', ],
#     ['oct18_r1.json', ],
#     ['oct18_r9.json', ] 
# ]

files = [['/home/adrian/dev/metrics/COCO_DATA/GT_COCO/Filtered/dec7_dhd1_short.mp4_COCO_Filtered.json',
          '/home/adrian/dev/metrics/COCO_DATA/detector_COCO/Filtered/yolov5m_dec7_dhd1_clean_short_Filtered.json']]

# cocoGts = []
# cocoDts = []
# cocoEvals = []
# for i, entry in enumerate(files):
#     tempGts = COCO(entry[0])
#     tempDts = COCO(entry[1])
#     cocoGts.append(tempGts)
#     cocoDts.append(tempGts.loadRes(tempDts))

#     catIds = cocoGts[i].getCatIds(catNms=['dog', 'cat', 'bird', 'car', 'bike', 'person', 'tree'])
#     imgIds = cocoGts[i].getImgIds(catIds=catIds)

#     cocoEvals[i] = COCOeval(cocoGts[i], cocoDts[i], 'bbox')
#     cocoEvals[i].params.catIds = catIds
#     cocoEvals[i].params.imgIds = imgIds

# for i, eval in enumerate(cocoEvals):
#   # Evaluate the predictions
#   eval.evaluate()
#   eval.accumulate()
#   eval.summarize()
#   print(eval.stats)

# Initialize an empty list to store the mAP values for each pair
mAP_list = []

# Loop through the data array
for pair in files:
    # Load the GT and Det json files using COCO
    coco_gt = COCO(pair[0]) # GT file
    coco_dt = coco_gt.loadRes(pair[1]) # Det file

    # Create a COCO evaluator object
    coco_eval = COCOeval(coco_gt, coco_dt, "bbox")

    # Evaluate the detections
    coco_eval.evaluate()
    coco_eval.accumulate()
    coco_eval.summarize()

    # Get the mAP value (the average of the AP values for each IoU threshold)
    mAP = np.mean(coco_eval.stats[:10])

    # Append the mAP value to the list
    mAP_list.append(mAP)

# Print the mAP list
print(mAP_list)