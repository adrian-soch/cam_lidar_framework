#! /usr/bin/env python3

"""

@file slim_track.py

@brief This is a slimmed down version of the original `track.py`.
    Assuming we will on use a single webcam, some redundant code has been removed.

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

import logging

from trackers.multi_tracker_zoo import create_tracker
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.dataloaders import LoadStreams
from yolov5.utils.general import (LOGGER, check_img_size, cv2,
                                  non_max_suppression, scale_coords)
from yolov5.utils.plots import Annotator, colors
from yolov5.utils.torch_utils import select_device, time_sync

# remove duplicated stream handler to avoid duplicated logging
logging.getLogger().removeHandler(logging.getLogger().handlers[0])

class VisionTracker():
    """
    Defines the class which creates a computer vision detection and tracking system.

    Source (input) is a webcam.
    """

    def __init__(self):
        """
        The VisionTracker initializer. Chage detection/tracking params here.

        @return An instance of the VisionTracker class with the specified name
        """
        source='2'
        yolo_weights=WEIGHTS / 'yolov5n.pt'  # model.pt path(s)
        reid_weights=WEIGHTS / 'osnet_x0_25_msmt17.pt'  # model.pt path

        imgsz=(640, 640)  # inference size (height, width)
        self.tracking_method='ocsort'
        self.conf_thres=0.25  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=255  # maximum detections per image
        self.device='cpu'  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.show_vid=True  # show results
        self.classes=[0, 1, 2, 3, 5, 7]  # filter by class: --class 0, or --class 0 2 3
        self.half=False  # use FP16 half-precision inference (GPU ONLY)
        dnn=False  # use OpenCV DNN for ONNX inference

        source = str(source)

        # Load model
        self.device = select_device(self.device)
        self.model = DetectMultiBackend(yolo_weights, device=self.device, dnn=dnn, data=None, fp16=self.half)
        stride, self.names, pt = self.model.stride, self.model.names, self.model.pt
        imgsz = check_img_size(imgsz, s=stride)  # check image size

        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)

        self.tracker = create_tracker(self.tracking_method, reid_weights, self.device, self.half)
        if hasattr(self.tracker, 'model'):
            if hasattr(self.tracker.model, 'warmup'):
                self.tracker.model.warmup()

        # Init variables for tracking loop
        self.dt = [0.0, 0.0, 0.0, 0.0]
    
    @torch.no_grad()
    def run(self):
        """
        Runs the detection and tracking loop

        @return None (loops through data)
        """

        outputs = [None]

        # Run tracking
        #model.warmup(imgsz=(1 if pt else nr_sources, 3, *imgsz))  # warmup

        curr_frame, prev_frame = [None], [None]

        # Iterate through the datastream
        for _, (_, im, im0s, _, s) in enumerate(self.dataset):
            t1 = time_sync()
            im = torch.from_numpy(im).to(self.device)
            im = im.half() if self.half else im.float()  # uint8 to fp16/32
            im /= 255.0  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim
            t2 = time_sync()
            self.dt[0] += t2 - t1

            # Inference
            pred = self.model(im, augment=False, visualize=False)
            t3 = time_sync()
            self.dt[1] += t3 - t2

            # Apply NMS
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, agnostic=False, max_det=self.max_det)
            self.dt[2] += time_sync() - t3

            # Process detections
            det = pred[0]
            im0 = im0s[0].copy()
            curr_frame = im0

            annotator = Annotator(im0, line_width=2, pil=not ascii)

            if hasattr(self.tracker, 'tracker') and hasattr(self.tracker.tracker, 'camera_update'):
                if prev_frame is not None and curr_frame is not None:  # camera motion compensation
                    self.tracker.tracker.camera_update(prev_frame, curr_frame)

            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()  # xyxy

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # pass detections to tracker
                t4 = time_sync()
                outputs = self.tracker.update(det.cpu(), im0)
                t5 = time_sync()
                self.dt[3] += t5 - t4

                # draw boxes for visualization
                if len(outputs) > 0:
                    for j, (output, conf) in enumerate(zip(outputs, det[:, 4])):
    
                        bboxes = output[0:4]
                        id = output[4]
                        cls = output[5]

                        if self.show_vid:  # Add bbox to image
                            c = int(cls)  # integer class
                            id = int(id)  # integer id
                            label = f'{id} {self.names[c]} {conf:.2f}'
                            annotator.box_label(bboxes, label, color=colors(c, True))
    
                LOGGER.info(f'{s}Done. yolo:({t3 - t2:.3f}s), {self.tracking_method}:({t5 - t4:.3f}s)')

            else:
                if self.tracking_method == 'strongsort':
                    self.tracker.increment_ages()
                LOGGER.info('No detections')

            if self.show_vid:
                # Stream results
                im0 = annotator.result()
                cv2.imshow('Detection+Tracker output', im0)
                cv2.waitKey(1)  # 1 millisecond

            prev_frame = curr_frame

if __name__ == "__main__":
    tracker = VisionTracker()
    tracker.run()