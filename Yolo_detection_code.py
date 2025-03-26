import torch
import cv2
import numpy as np
from ultralytics import YOLO



# Load the custom-trained YOLOv8 model
model = YOLO('/Users/yuthishkumar/Downloads/best_70_epochs_yolov8_4_class.pt')  # Path to your YOLOv8 model


device = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(device)

# Function to perform detection with confidence filtering for all classes
def detect_objects_on_frame(frame, confidence_threshold=0.7):
    # Perform inference
    results = model(frame)  # Directly pass the frame

    # Loop over all detected objects
    for result in results:
        boxes = result.boxes  # Extract the bounding boxes and other information
        
        for box in boxes:
            class_id = int(box.cls[0])  # Class index
            confidence = box.conf[0].item()  # Confidence score
            class_name = model.names[class_id]  # Get the class name

            # Apply confidence threshold filtering
            if confidence >= confidence_threshold:
                # Get bounding box coordinates
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0].tolist())

                # Draw bounding box and label on the frame
                top_left = (x_min, y_min)
                bottom_right = (x_max, y_max)
                color = (0, 255, 0)  # Green box by default
                if class_name == 'rabbit':
                    color = (0, 255, 0)  # Green
                elif class_name == 'face':
                    color = (255, 0, 0)  # Blue
                elif class_name == 'cat':
                    color = (0, 0, 255)  # Red
                elif class_name == 'bird':
                    color = (255, 255, 0)  # Cyan
                
                cv2.rectangle(frame, top_left, bottom_right, color, 2)
                label = f"{class_name} ({confidence:.2f})"
                cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    return frame

# Open the webcam and perform live detection
cap = cv2.VideoCapture(0)  # 0 is typically the default webcam

if not cap.isOpened():
    print("Error: Could not open webcam.")
else:
    print("Starting live object detection. Press 'q' to exit.")

    while True:
        # Capture frame-by-frame from webcam
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame.")
            break

        # Perform object detection on the frame
        frame_with_boxes = detect_objects_on_frame(frame)

        # Display the resulting frame with bounding boxes
        cv2.imshow('Live Object Detection', frame_with_boxes)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When everything is done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()
