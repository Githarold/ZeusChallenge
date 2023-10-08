import cv2
import torch
import threading
import numpy as np
from utils.plots import plot_one_box
from utils.datasets import LoadImages
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords

class YoloDetector:
    def __init__(self, weights_path="yolo7.pt"):
        # Initialization code
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(weights_path, map_location=self.device)  # load FP32 model
        self.imgsz = check_img_size(640, s=self.model.stride.max())  # check img_size
        if self.half:
            self.model.half()  # to FP16

        # Load class names
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names

    def detect(self, frame, save_img=False):
        # Check if the frame is a NumPy array, if not, convert it
        if not isinstance(frame, np.ndarray):
            frame = np.asanyarray(frame.get_data())
        
        # Use the provided RGB frame for detection
        im0 = frame.copy()  # Copy to avoid modifying the original frame
        img = torch.from_numpy(frame).permute(2, 0, 1).to(self.device).float() / 255.0

        if self.half:
            img = img.half()

        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        pred = self.model(img)[0]
        pred = non_max_suppression(pred)

        detected_classes = {}

        # Process detections
        if len(pred[0]):
            det = pred[0]
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
            for *xyxy, conf, cls in reversed(det):
                label = f"{self.names[int(cls)]} {conf:.2f}"
                plot_one_box(xyxy, im0, label=label, color=(255, 0, 0), line_thickness=3)
                detected_classes[int(cls)] = detected_classes.get(int(cls), 0) + 1
        
        # Display the image with detections in a separate thread
        threading.Thread(target=self.display_image, args=(im0,)).start()

        return detected_classes
    
    def display_image(self, img):
        
        cv2.imshow('Detection', img)
        cv2.waitKey(5000)  # Wait for 5 seconds
        cv2.destroyAllWindows()
        