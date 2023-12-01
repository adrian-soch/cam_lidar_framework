from PIL import Image, ImageDraw
from ultralytics import YOLO

img_path = "/home/adrian/dev/bags/cleaned_bags/may10_q7_rebag/images/000046_1683739422_296893652.jpg"

model = YOLO("yolov8m-seg.pt")
results = model.predict(img_path, save=True, imgsz=640, conf=0.5, device='0')
result = results[0]

masks = result.masks
mask1 = masks[0]

mask = mask1.data[0].cpu().numpy()
polygon = mask1.xy[0]

img = Image.open(img_path)
draw = ImageDraw.Draw(img)
draw.polygon(polygon,outline=(0,255,0), width=5)
img.show()

mask_img = Image.fromarray(mask,"I")
mask_img.show()
