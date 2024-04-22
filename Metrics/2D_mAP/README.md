# COCO mAP Calculation

1. Get a COCO format ground truth. For superviesly video annotations, use the `supervisely_video_ann2coco.py` script.
2. Get the detector COCO results. The `cam_lidar_tools/camera_det2d/camera_det2d/camera_processing_node.py` has a built in flag (`SAVE_COCO_JSON`) to save results to a .json file in the folder based on `JSON_PATH`.
3. Filter the detection results for to contain only results inside the RoI for each data sequence.
   1.  Use `filter_coco_detections.py`, set the RoI points (same as `cam_lidar_tools/Metrics/2D_HOTA/preprocessGT.py` used for HOTA).
4.  Calcualte the COCO mAP using `coco_map_calc.py`. Set the files list to contain the set of GT and Detection files as desired.
