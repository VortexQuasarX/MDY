import serial
import time
from ultralytics import YOLO
import cv2

# Load the YOLO model
model = YOLO('/Users/yuthishkumar/Downloads/best_70_epochs_yolov8_4_class.pt')

# Connect to Arduino
arduino = serial.Serial('/dev/cu.usbmodem101', 9600)  # Replace with your actual port

def detect_objects(frame):
    results = model(frame)
    detected_classes = []

    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            class_name = model.names[class_id]
            confidence = box.conf[0].item()
            
            # Only consider detections with confidence >= 0.6
            if confidence >= 0.6:
                # Draw bounding box and label on the frame
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0].tolist())
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)  # Green box
                label = f"{class_name} ({confidence:.2f})"
                cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Track detected classes
                detected_classes.append(class_name)

    return frame, detected_classes

# Video capture loop
cap = cv2.VideoCapture(0)  # Webcam input
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Perform detection and draw bounding boxes
    frame_with_detections, detected_classes = detect_objects(frame)

    # Send commands to Arduino based on detected classes
    if "bird" in detected_classes:
        arduino.write(b'BUZZER_ON\n')
        arduino.write(b'LIGHT_ON\n')
          # Turn on light for bird detection
    else:
        arduino.write(b'LIGHT_OFF\n')  # Turn off light if no bird detected

    if "cat" in detected_classes:
        
        arduino.write(b'ELECTRIC_MAT_ON\n')
        arduino.write(b'VIBRATION_EMITTER_ON\n')
          # Turn on humidifier for cat detection
    else:
        
        arduino.write(b'ELECTRIC_MAT_OFF\n')
        arduino.write(b'VIBRATION_EMITTER_OFF\n')  # Turn off humidifier if no cat detected

    if "rabbit" in detected_classes:
 
        arduino.write(b'HUMIDIFIER_ON\n')  # Turn on vibration emitter for rabbit
        arduino.write(b'BUZZER_ON\n')  # Activate buzzer
        time.sleep(1)  # Delay to avoid duplicate commands
    else:
          # Turn off vibration emitter if no rabbit detected
        arduino.write(b'HUMIDIFIER_OFF\n')  # Turn off electric mat if no rabbit detected

    # Display frame with bounding boxes
    cv2.imshow("Detection", frame_with_detections)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()