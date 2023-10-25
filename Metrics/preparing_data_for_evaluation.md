# Preparing Data for Evalutation

## Background

The .txt files that contain fusion and tracker output from the ROS system require some prep before evaulating with metrics.
- HOTA requires frame numbers to start from 1.
- GT and Tracker data must have the same number of frames.

## Steps

1. **Re-number frames**
   - Re-number the frames starting from 1
   - Useful if you deleted some starting frames to keep the size the same (**see note below**)
2. **Preprocess detections**
   - Filter out detections that are not inside a region of interest

Use `preprocessGT.py`, you can set the ROI as a list of points that create a polygon and also view which detections are kept on a sample image. It will also re-number the frames automatically for you.

If you just want to re-number the frames without prerocessing you can run the `re_number_frames.py` script.

> **Note**: Larger detection netowrks have larger start up delays and may miss several of the starting image frames. To keep the same number of frames in GT and for each tracker, keep the last X frames that allow each tracker result to have the same number of frames. For example GT has frames 1-66, and tracker1 has frames 1-65 you can delete the frist 5 and 6 frames respectively.

**Example Output:** Red bboxes are removes, green are retained, and the blue outline is the RoI. <p align="center"><img src="../Docs/readme_images/fusion_detection_visualization.png" alt="drawing" width="700"/></p>