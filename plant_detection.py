import torch
import cv2
import numpy as np
from picamera2 import Picamera2
from torchvision import transforms
from PIL import Image

from ultralytics import YOLO

# Load the best model weights
model_path = "YOLOv11/runs/detect/train5/weights/best.pt"  # Path to the best model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = YOLO(model_path)

# Initialize Picamera2
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (1280, 720)})
picam2.configure(camera_config)
picam2.start()

def detect_objects(frame):
    # Convert frame from BGR (OpenCV) to RGB
    # image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # img = Image.fromarray(image)
    img = Image.fromarray(frame)

    # Perform inference
    results = model.predict(source=img, save=True)
    
    plant_detected = False
    pot_detected = False
    x_plant, y_plant, x_pot, y_pot = 0, 0, 0, 0
    w_plant, h_plant, w_pot, h_pot = 0, 0, 0, 0
    condition = 0
    for result in results:
        # Extract predictions
        detections = result.boxes.xywh.cpu().numpy()  # xywh format: [x_center, y_center, width, height]
        confidences = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy()  # class id = 0: plant, 1: pot
        
        for detection, confidence, class_id in zip(detections, confidences, class_ids):
            x_center, y_center, width, height = detection
            conf = confidence

            if class_id == 1 and conf > 0.5:  # Pot
                pot_detected = True
                x_pot, x_pot = x_center, y_center
                w_pot, h_pot = width, height
                condition = w_pot * 2
                print(f"Pot -> X: {x_pot:.2f}, Y: {y_pot:.2f}, "
                      f"W: {width:.2f}, H: {height:.2f}, Confidence: {conf:.2f}")

            elif class_id == 0 and conf > 0.5 and width < condition:  # Plant
                plant_detected = True
                x_plant, y_plant = x_center, y_center
                print(f"Plant -> X: {x_plant:.2f}, Y: {y_center:.2f}, "
                      f"W: {width:.2f}, H: {height:.2f}, Confidence: {conf:.2f}")
            
    
    if plant_detected and pot_detected:
        print("Detected potted plant!")
        picam2.stop()
        print("Camera stopped.")
        exit()

try:
    print("Starting object detection...")
    
    while True:
        # Capture a frame
        frame = picam2.capture_array()

        # Perform object detection
        detect_objects(frame)

        # Add a small delay
        cv2.waitKey(10)

except KeyboardInterrupt:
    print("Detection stopped by user.")
finally:
    picam2.stop()
    print("Camera stopped.")
