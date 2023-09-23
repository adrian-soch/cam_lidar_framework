#! /usr/bin/env python3

"""

@file vision_track.py

@brief This is based from `slim_track.py`, it has been converted to init a detector/tracker
    and then call `update()` with a new image inside a loop.

@section TODO
- Seperate showing the video result into another thread beacuase it adds a large variable delay to the loop

@section Author(s)
- Created by Adrian Sochaniwsky on 13/11/2022

"""

import os

# limit the number of cpus used by high performance libraries
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "2"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import sys
from pathlib import Path

import numpy as np
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolov5 strongsort root directory
WEIGHTS = ROOT / 'weights'

if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
if str(ROOT / 'yolov5') not in sys.path:
    sys.path.append(str(ROOT / 'yolov5'))  # add yolov5 ROOT to PATH
if str(ROOT / 'trackers' / 'strong_sort') not in sys.path:
    sys.path.append(str(ROOT / 'trackers' / 'strong_sort'))  # add strong_sort ROOT to PATH

ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from trackers.multi_tracker_zoo import create_tracker
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.augmentations import letterbox
from yolov5.utils.general import (check_img_size, cv2,
                                  non_max_suppression, scale_boxes)
from yolov5.utils.plots import Annotator, colors
from yolov5.utils.torch_utils import time_sync, select_device

class VisionTracker():
    """
    Defines the class which creates a computer vision detection and tracking system.

    Source (input) is an image
    """

    def __init__(self):
        """
        The VisionTracker initializer. Chage detection/tracking params here.

        @return An instance of the VisionTracker class with the specified name
        """

        yolo_weights=WEIGHTS / 'yolov5s.pt'  # model.pt path(s)
        reid_weights=WEIGHTS / 'osnet_x0_25_msmt17.pt'  # model.pt path

        self.imgsz=(640, 640)  # inference size (height, width)
        self.tracking_method='ocsort' # ocsort or strongsort
        self.conf_thres=0.25  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=128  # maximum detections per image
        self.device='0'  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.show_vid=False  # show results
        self.classes=[0, 1, 2, 3, 5, 7]  # filter by class: --class 0, or --class 0 2 3
        self.half=True  # use FP16 half-precision inference (GPU ONLY)
        dnn=False  # use OpenCV DNN for ONNX inference

        # Load model
        self.device = select_device(self.device)
        self.model = DetectMultiBackend(yolo_weights, device=self.device, dnn=dnn, data=None, fp16=self.half)
        stride, self.names, _ = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(self.imgsz, s=stride)  # check image size
        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Create tracker object
        self.tracker = create_tracker(self.tracking_method, reid_weights, self.device, self.half, config_path_root=str(ROOT))
        if hasattr(self.tracker, 'model'):
            if hasattr(self.tracker.model, 'warmup'):
                self.tracker.model.warmup()

        # Init variables for tracking loop
        self.curr_frame, self.prev_frame = [None], [None]
    
    @torch.no_grad()
    def update(self, im, return_image=False):
        """
        Runs the detection and tracking, must be called for each image.
        Developer muat ensure loop speed is sufficient for desired output frequency

        @param im Input frame from to be processed

        @return detections and tracker output
        """
        outputs = [None]

        # Preprocess image for model
        im0 = im.copy()
        im = letterbox(im, self.imgsz, stride=32, auto=True)[0]
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)

        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()  # uint8 to fp16/32
        im /= 255.0  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

        # Inference
        pred = self.model(im, augment=False, visualize=False)

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, agnostic=False, max_det=self.max_det)
        # self.dt[2] += time_sync() - t3
        t4 = time_sync()

        annotator = Annotator(im0, line_width=2, pil=not ascii)
        #annotator = Annotator(im0, line_width=2)

        if hasattr(self.tracker, 'tracker') and hasattr(self.tracker.tracker, 'camera_update'):
            if self.prev_frame is not None and self.curr_frame is not None:  # camera motion compensation
                self.tracker.tracker.camera_update(self.prev_frame, self.curr_frame)

        # Process detections
        det = pred[0]
        self.curr_frame = im0

        if det is not None and len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()  # xyxy

            # pass detections to tracker
            outputs = self.tracker.update(det.cpu(), im0)

            # draw boxes for visualization
            if len(outputs) > 0:
                for j, (output, conf) in enumerate(zip(outputs, det[:, 4])):

                    bboxes = output[0:4]
                    id = output[4]
                    cls = output[5]

                    if self.show_vid or return_image:  # Add bbox to image
                        c = int(cls)  # integer class
                        id = int(id)  # integer id
                        label = f'{id} {self.names[c]} {conf:.2f}'
                        annotator.box_label(bboxes, label, color=colors(c, True))
        else:
            empty_tensor = torch.empty((0, 6))
            self.tracker.update(empty_tensor, im0)

        if self.show_vid or return_image:
            # Stream results
            im0 = annotator.result()
            
        if self.show_vid:
            cv2.imshow('Detection+Tracker output', im0)
            cv2.waitKey(1)  # 1 millisecond

        self.prev_frame = self.curr_frame
        t7 = time_sync()
        # LOGGER.info(f'Pre: {t2-t1}, Inf: {t3-t2}, NMS: {t4-t3}, CreA: {t5-t4}, Draw {t6-t5}, Disp: {t7-t6} Final = {t7-t1}')

        if return_image:
            return outputs, im0
        else:
            return outputs
