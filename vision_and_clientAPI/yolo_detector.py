import cv2
import torch
import threading
import numpy as np
import pyrealsense2 as rs
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

    # to make weight map
    def detect_obj_property(self, color_frame, depth_frame, depth_scale, intrinsics):

        detected_objects = []

        # Check if the frame is a NumPy array, if not, convert it
        if not isinstance(color_frame, np.ndarray):
            color_frame = np.asanyarray(color_frame.get_data())
        
        # Use the provided RGB frame for detection
        im0 = color_frame.copy()  # Copy to avoid modifying the original frame
        img = torch.from_numpy(color_frame).permute(2, 0, 1).to(self.device).float() / 255.0

        depth_image = np.asanyarray(depth_frame.get_data())

        if self.half:
            img = img.half()

        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        pred = self.model(img)[0]
        pred = non_max_suppression(pred)

        # Process detections
        if len(pred[0]):
            det = pred[0]
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
            for *xyxy, conf, cls in reversed(det):
                x1, y1, x2, y2 = xyxy
                x2 = min(int(x2), depth_image.shape[1] - 1)
                y2 = min(int(y2), depth_image.shape[0] - 1)
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                center_x = center_x.cpu().detach().numpy()
                center_y = center_y.cpu().detach().numpy()
                depth_value = depth_image[int(center_y)][int(center_x)] * depth_scale

                x3D_1, y3D_1, z3D_1 = rs.rs2_deproject_pixel_to_point(intrinsics, [x1, y1], depth_value)
                x3D_2, y3D_2, z3D_2 = rs.rs2_deproject_pixel_to_point(intrinsics, [x2, y2], depth_value)
                real_width = (abs(x3D_2 - x3D_1))
                real_height = (abs(y3D_2 - y3D_1))

                weight = self.class2weight(int(cls))

                # print(f"Object {weight}: Real Width: {int(real_width*100)}, Real Height: {int(real_height*100)}, Top-left corner: ({int(x3D_1*100)}, {int(y3D_1*100)})")
                detected_objects.append([(int(x3D_1*100)), (int(y3D_1*100)), int(real_width*100), int(real_height*100), weight])


                label = f"{self.names[int(cls)]} {conf:.2f}"
                plot_one_box(xyxy, im0, label=label, color=(255, 0, 0), line_thickness=3)
        
        # Display the image with detections in a separate thread
        # threading.Thread(target=self.display_image, args=(im0,)).start()

        return np.array(detected_objects)
    
    # anygrasp obj detect or for fixed cam
    def detect_obj_list(self, color_frame):

        flag = 0
        detected_classes = [] 
        # Check if the frame is a NumPy array, if not, convert it
        if not isinstance(color_frame, np.ndarray):
            color_frame = np.asanyarray(color_frame.get_data())

        
        # Use the provided RGB frame for detection
        im0 = color_frame.copy()  # Copy to avoid modifying the original frame
        img = torch.from_numpy(color_frame).permute(2, 0, 1).to(self.device).float() / 255.0

        if self.half:
            img = img.half()


        if img.ndimension() == 3:
            img = img.unsqueeze(0)


        # Inference
        pred = self.model(img)[0]
        pred = non_max_suppression(pred)


        # Process detections
        if len(pred[0]):
            flag = 1
            det = pred[0]
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
            for *xyxy, conf, cls in reversed(det):
                label = f"{self.names[int(cls)]} {conf:.2f}"
                plot_one_box(xyxy, im0, label=label, color=(255, 0, 0), line_thickness=3)
                # detected_classes[int(cls)] = detected_classes.get(int(cls), 0) + 1
                weight = self.class2weight(int(cls))
                detected_classes.append(weight)


        
        # Display the image with detections in a separate thread
        threading.Thread(target=self.display_image, args=(im0,)).start()

        return np.array(detected_classes), flag
    
    def display_image(self, img):
        
        cv2.imshow('Detection', img)
        cv2.waitKey(5000)  # Wait for 5 seconds
        cv2.destroyAllWindows()
    
    def class2weight(self, cls): # [[1]]
        # jelly = 0, milk_tea = 1, tissue = 2, cup_ramen = 3, spam = 4, choco_milk = 5, ace = 6
        print("obj list1")

        if cls in [1, 2, 3, 4]:
            weight = 2
        else:
            weight = 1
        print("obj list2")
            
        return weight