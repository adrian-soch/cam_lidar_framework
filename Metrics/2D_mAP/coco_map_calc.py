from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval

NUM_GT_FILES = 6

# [GT File, Det File]
files = [
    # camera-only
    ['dec7_dhd1.json', ],
    ['dec14_ok.json', ],
    ['may10_q5.json', ],
    ['may10_q7.json', ],
    ['oct18_r1.json', ],
    ['oct18_r9.json', ],
    # lidar-only
    ['dec7_dhd1.json', ],
    ['dec14_ok.json', ],
    ['may10_q5.json', ],
    ['may10_q7.json', ],
    ['oct18_r1.json', ],
    ['oct18_r9.json', ],
    # camera-lidar
    ['dec7_dhd1.json', ],
    ['dec14_ok.json', ],
    ['may10_q5.json', ],
    ['may10_q7.json', ],
    ['oct18_r1.json', ],
    ['oct18_r9.json', ] 
]

cocoGts = []
cocoDts = []
cocoEvals = []
for i, entry in enumerate(files):
    cocoGts[i] = COCO(entry[i][0])
    cocoDts[i] = cocoGts[i].loadRes(entry[i][1])

    catIds = cocoGts[i].getCatIds(catNms=['dog', 'cat', 'bird', 'car', 'bike', 'person', 'tree'])
    imgIds = cocoGts[i].getImgIds(catIds=catIds)

    cocoEvals[i] = COCOeval(cocoGts[i], cocoDts[i], 'bbox')
    cocoEvals[i].params.catIds = catIds
    cocoEvals[i].params.imgIds = imgIds

for i, eval in enumerate(cocoEvals):
  # Evaluate the predictions
  eval.evaluate()
  eval.accumulate()
  eval.summarize()
  print(eval.stats)
