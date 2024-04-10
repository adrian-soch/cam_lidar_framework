# from ultralytics.models.yolo.obb import OBBTrainer

# args = dict(model='yolov8n-obb.pt', data='a9_obb.yaml', epochs=1)
# trainer = OBBTrainer(overrides=args)
# trainer.train()



from ultralytics import YOLO
from torch.utils.data import DataLoader

from a9_dataset import A9LidarBevCreator

# Create a model from YAML file
model = YOLO('yolov8n.yaml')

# Load pretrained weights (optional)
model.load('yolov8n.pt')

train_dataloader = DataLoader(A9LidarBevCreator, batch_size=configs.batch_size, shuffle=(train_sampler is None),
                                  pin_memory=configs.pin_memory, num_workers=configs.num_workers, sampler=train_sampler)

# Create a custom dataloader object
dataloader = CustomDataLoader(...)

# Train the model
results = model.train(data=dataloader, epochs=100, imgsz=640)