from ultralytics import YOLO

# Load a model, if an offical model it will auto-download
# if custom model, provide full path
model = YOLO('yolov8m-seg.pt')

# Export the model
# see here for options: https://docs.ultralytics.com/modes/export/#arguments
model.export(format='engine', imgsz=640, batch=1, half=True, simplify=False)