#! /usr/bin/env python3
"""

@file vision_track_yolov7.py

@brief This is based from `vision_track.py`, it has been converted to use yolov7

@section Author(s)
- Created by Adrian Sochaniwsky on 23/12/2022

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

import cv2

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolov7 strongsort root directory
WEIGHTS = ROOT / 'weights'

if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
if str(ROOT / 'yolov7') not in sys.path:
    sys.path.append(str(ROOT / 'yolov7'))  # add yolov7 ROOT to PATH
# if str(ROOT / 'yolov5') not in sys.path:
#     sys.path.append(str(ROOT / 'yolov5'))  # add yolov5 ROOT to PATH
if str(ROOT / 'trackers' / 'strong_sort') not in sys.path:
    sys.path.append(str(ROOT / 'trackers' / 'strong_sort'))  # add strong_sort ROOT to PATH

ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from trackers.multi_tracker_zoo import create_tracker

from yolov7.models.experimental import attempt_load
from yolov7.utils.general import check_img_size, non_max_suppression, \
    scale_coords, set_logging
from yolov7.utils.plots import plot_one_box
from yolov7.utils.torch_utils import select_device, time_synchronized
from yolov7.utils.datasets import letterbox

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

        yolo_weights=WEIGHTS / 'yolov7.pt'  # model.pt path(s)
        reid_weights=WEIGHTS / 'osnet_x0_25_msmt17.pt'  # model.pt path

        self.flip = True # Flip image about x-axis (about the horizon)
        self.imgsz=(640, 640)  # inference size (height, width)
        self.tracking_method='ocsort' # ocsort or strongsort
        self.conf_thres=0.25  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=255  # maximum detections per image
        self.device='0'  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.show_vid=False  # show results
        self.classes=[0, 1, 2, 3, 5, 7]  # filter by class: --class 0, or --class 0 2 3
        self.half=True  # use FP16 half-precision inference (GPU ONLY)
        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Initialize
        set_logging()
        self.device = select_device(self.device)
        half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(yolo_weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(self.imgsz[0], s=self.stride)  # check img_size

        if half:
            self.model.half()  # to FP16

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # run once
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))

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
        if self.flip:
            im = cv2.flip(im, -1)

        im0 = im.copy()
        im = letterbox(im, new_shape=self.imgsz, stride=32, auto=True)[0]


        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)


        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()  # uint8 to fp16/32
        im /= 255.0  # 0 - 255 to 0.0 - 1.0
        if im.ndimension() == 3:
            im = im.unsqueeze(0)

        # Inference
        pred = self.model(im, augment=False)[0]

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, None, agnostic=False)
        # pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, agnostic=False, max_det=self.max_det)
        # self.dt[2] += time_synchronized() - t3
        t4 = time_synchronized()

        if hasattr(self.tracker, 'tracker') and hasattr(self.tracker.tracker, 'camera_update'):
            if self.prev_frame is not None and self.curr_frame is not None:  # camera motion compensation
                self.tracker.tracker.camera_update(self.prev_frame, self.curr_frame)

        # Process detections
        det = pred[0]
        self.curr_frame = im0

        if det is not None and len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Print results
            s = ''
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # pass detections to tracker
            outputs = self.tracker.update(det.cpu(), im0)

            # Write results
            for *xyxy, conf, cls in reversed(det):
                if self.show_vid or return_image:  # Add bbox to image
                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=2)
        else:
            self.tracker.update(np.empty((0, 5)), im0)
            # if self.tracking_method == 'strongsort':
            #     self.tracker.increment_ages()
            pass

        if self.show_vid:
            cv2.imshow('Detection+Tracker output', im0)
            cv2.waitKey(1)  # 1 millisecond

        self.prev_frame = self.curr_frame
        t7 = time_synchronized()
        # LOGGER.info(f'Pre: {t2-t1}, Inf: {t3-t2}, NMS: {t4-t3}, CreA: {t5-t4}, Draw {t6-t5}, Disp: {t7-t6} Final = {t7-t1}')

        if return_image:
            return outputs, im0
        else:
            return outputs
