# Animal Detection and Deterrent System

This project is a comprehensive system for detecting animals in real-time using a YOLOv8 object detection model and activating specific hardware deterrents with an Arduino. It's designed to be a flexible and effective solution for protecting gardens, property, or any area from unwanted animal presence.

![Circuit Diagram](Circuit%20Diagrram%20for%20Arduino%20and%20Deterrents.png)

## 🌟 Features

*   **Real-time Animal Detection:** Utilizes a fine-tuned YOLOv8 model to detect multiple animal classes.
*   **Customizable Deterrents:** Triggers different hardware deterrents based on the detected animal.
*   **Modular Architecture:** The system is divided into a Python-based detection module and an Arduino-based hardware control module, allowing for easy customization and expansion.
*   **Pre-trained Model Included:** Comes with a pre-trained model for detecting cats, birds, and rabbits.
*   **Training Notebook:** Includes a Jupyter notebook to allow for training the model on your own custom dataset.

## 🏗️ System Architecture

The system operates in a continuous loop:

1.  **Image Capture:** The Python script captures video frames from a webcam.
2.  **Object Detection:** Each frame is passed to the YOLOv8 model for inference.
3.  **Command Generation:** If an animal is detected with a confidence score above a set threshold (currently 0.6), the script sends a corresponding command to the Arduino via a serial connection.
4.  **Deterrent Activation:** The Arduino receives the command and activates the appropriate hardware deterrent(s).

![Flowchart](Animal%20Detection%20and%20Deterrent%20Activation%20table.png)

## 🛠️ Hardware Requirements

*   Arduino Uno (or a compatible board)
*   Webcam
*   5V 4-Channel Relay Module
*   **Deterrents:**
    *   Light Bulb (for birds)
    *   Vibration Emitter (for cats)
    *   Electric Mat (for cats)
    *   Humidifier/Sprinkler (for rabbits)
    *   Buzzer
*   Jumper Wires
*   Breadboard
*   External power supply for the relay module (recommended)

## 💻 Software and Dependencies

*   Python 3.8+
*   Arduino IDE
*   Required Python libraries:
    *   `ultralytics`
    *   `opencv-python`
    *   `pyserial`

## 🚀 Setup and Installation

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Python Environment Setup

It is recommended to use a virtual environment.

```bash
python -m venv venv
source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
```

Install the required Python libraries:

```bash
pip install ultralytics opencv-python pyserial
```

### 3. Arduino Setup

1.  Open the `arduino_deterrent_setup.ino` file in the Arduino IDE.
2.  **Connect your Arduino** to your computer.
3.  Select the correct board and port from the `Tools` menu.
4.  **⚠️ Bug Fix:** Before uploading, you must fix a bug in the code. In the `loop()` function, find this line:
    ```cpp
    if (command == “BUZZER_ON”) {
    ```
    The quotes around `BUZZER_ON` are "smart quotes" and will cause a compilation error. Change them to standard double quotes:
    ```cpp
    if (command == "BUZZER_ON") {
    ```
5.  Click the **Upload** button to flash the sketch to your Arduino.

### 4. Code Configuration

You need to update the hardcoded paths in the project files to match your system.

1.  **Detection Script (`Detection_&_sending_Arduino_commands.py`)**:
    *   On line 6, change the model path to the relative path:
      ```python
      model = YOLO('YOLO_v8_pretrained_model.pt')
      ```
    *   On line 9, change the Arduino serial port to match the one your Arduino is connected to. You can find this in the Arduino IDE under `Tools > Port`.
      ```python
      arduino = serial.Serial('/dev/your_arduino_port', 9600)
      ```

2.  **Dataset Configuration (`data.yaml`)**:
    *   If you plan to retrain the model, update the paths in `data.yaml` to be relative:
      ```yaml
      train: ./images/train
      val: ./images/val
      ```

## ▶️ How to Use

Once the setup is complete, you can run the main detection script:

```bash
python Detection_&_sending_Arduino_commands.py
```

A window will open showing the webcam feed with bounding boxes around detected animals. When an animal is detected, the corresponding deterrents will be activated. Press 'q' to quit.

## 🧠 Training Your Own Model

You can fine-tune the model or train it on a completely new dataset using the provided Jupyter Notebook.

1.  **Prepare your dataset:** Your dataset should be in the YOLO format, with `images` and `labels` directories for your training and validation sets.
2.  **Update `data.yaml`:** Modify the `data.yaml` file to point to your dataset directories and list your custom class names.
3.  **Run the notebook:** Open and run the cells in `YOLOv8_training.ipynb`. The notebook will guide you through the training process, and the best trained model will be saved in the `runs/detect/train/weights/` directory. You can then use this new model in the detection script.

## 📊 Results

The included model was trained for 60 epochs and achieves excellent performance on the test dataset.

| Metric        | Value |
|---------------|-------|
| mAP50-95      | 0.821 |
| mAP50         | 0.974 |

Here are some performance graphs from the training process:

| Confusion Matrix                               | Precision-Recall Curve                           | ROC Curve                           |
|------------------------------------------------|--------------------------------------------------|-------------------------------------|
| ![Confusion Matrix](Confusion%20Matrix.png) | ![Precision-Recall Curve](Precision-Recall%20Curve.png) | ![ROC Curve](ROC%20Curve.png) |

## 🔍 Known Issues and Improvements

*   **Deterrent Behavior:** There is a discrepancy between the Python script's apparent intent and the Arduino's execution. The light deterrent is a *brief flash* and the buzzer is a *short pulse*, not continuous as long as the animal is present. The code can be modified to support continuous activation if needed.
*   **Flickering Deterrents:** The logic for turning deterrents off is handled in `else` blocks in the Python script. If a detection is not stable across consecutive frames, this can cause the deterrents to rapidly turn on and off. A more robust approach would be to use a timer-based system (e.g., keep the deterrent on for a few seconds after the last detection).
