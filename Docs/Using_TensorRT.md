# Using TensorRT for faster model inference

If using an Nvidia GPU it is likely that converting a PyTorch model (`.pt`) to a TensorRT compatable model (`.engine`) will yield a significant speed up in inferences.

Caveats:
- This is for constant size images.
- Speed ups are more significant for larger model (i.e. yolov5l)

## Instructions

This is an easy process, with the script provided from the `Yolov5` repo (https://github.com/ultralytics/yolov5).

1. Clone and `cd` into the yolov5 repo
2. Run the following command
```
python3 export.py --weights <name of python model weights> --include engine --half --device <gpu number> --img <image_height> <image_width>
```
Example command for yolov5 Large network, with half precision (for more speed), a `384x640` image, and 1 GPU with a device name of `0`.

```
python export.py --weights yolov5l.pt --include engine --half --device 0 --img 384 640
```
