import os
import cv2
from ultralytics import YOLO
import time
# Path to the folder containing images
IMAGES_DIR = '/home/charu_ws/yolo_detection/dataset/val/images/'
time_elapsed = []
# Initialize YOLO model
model_path = '/home/charu_ws/yolo_detection/experiments/pallet_detection_model_v1.0.8/weights/best.pt'
model = YOLO(model_path)

# Threshold for object detection
threshold = 0.3

# Loop through each image in the folder
for image_name in os.listdir(IMAGES_DIR):
    st = time.time()
    if image_name.endswith('.jpg') or image_name.endswith('.png'):
        image_path = os.path.join(IMAGES_DIR, image_name)
        frame = cv2.imread(image_path)
        H, W, _ = frame.shape

        # Perform object detection
        results = model(frame)[0]

        # Process detection results
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

            if score > threshold:
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
                cv2.putText(frame, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)

        # Save annotated image with detections
        annotated_image_path = os.path.join(IMAGES_DIR, f"{os.path.splitext(image_name)[0]}_annotated.jpg")
        cv2.imwrite(annotated_image_path, frame)
    et = time.time()
    elapsed_time = et - st
    time_elapsed.append(elapsed_time)


cv2.destroyAllWindows()