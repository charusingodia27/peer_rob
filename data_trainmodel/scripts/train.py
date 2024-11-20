from ultralytics import YOLO

# Load the pretrained model from the previous run
model = YOLO('yolov8s.pt')  # load previous weights

# Retrain the model
results = model.train(
    data='scripts/data.yaml',
    epochs=500,  # You can set the number of additional epochs here
    imgsz=416,
    batch=16,
    # resume=True,
    project='experiments',
    name='pallet_detection_model_v1.0.8'  # Change name to avoid overwriting previous run
)

# # Evaluate the model's performance
# results = model.val()

# Perform object detection on an image (optional)
results = model('CharuDataset/val/images/1564563279-4213042_jpg.rf.1bc800024c0ddd6f2ebcb330931ad81b.jpg')

# # Export the retrained model to ONNX format
# success = model.export(format='onnx')
