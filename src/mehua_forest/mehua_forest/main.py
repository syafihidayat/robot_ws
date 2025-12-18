import cv2
import numpy as np
from .cam import ObjectDetectionMapper 
from .bfs import PathFinder 

class ObjectDetectionMapper:
    def __init__(self, field_width=3, field_height=3, cam_id=0):
        self.field_width = field_width
        self.field_height = field_height
        self.grid = np.full((field_height, field_width), 'Empty', dtype=object)
        self.detected_objects = []
        self.robot_position = (0,0)

        # Load model YOLO (.pt bukan zip lagi)
        self.model = YOLO("/home/syafihidayat/Documents/robot_ws/src/mehua_forest/models/yolo11n.pt")

        # Kamera
        self.cam = cv2.VideoCapture(cam_id)

        self.target_classes = {
            'real'   : 'Target',
            'fake'   : 'Forbidden',
            'symbol' : 'Forbidden'
        }

    # =================== CAMERA DETECT SEKALI ===================== #
    def capture_and_detect_once(self):
        ret, frame = self.cam.read()
        if not ret:
            print("❌ Kamera tidak bisa dibuka / tidak terdeteksi!")
            return None, []

        results = self.model(frame)[0]
        detected = []

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1,y1,x2,y2 = map(int, box.xyxy[0])
            class_name = self.model.names[cls_id]

            obj_type = self.target_classes.get(class_name, "Unknown")
            detected.append({
                'class_name': class_name,
                'type': obj_type,
                'confidence': conf,
                'bbox': (x1,y1,x2,y2)
            })

        self.detected_objects = detected
        return frame, detected

    # ================== GRID MAPPING ================== #
    def map_to_grid(self, detected_objects, frame_size):
        w,h = frame_size[1], frame_size[0]
        cell_w = w / self.field_width
        cell_h = h / self.field_height

        for obj in detected_objects:
            x1,y1,x2,y2 = obj['bbox']
            center_x = (x1+x2)/2
            center_y = (y1+y2)/2

            grid_x = int(center_x // cell_w)
            grid_y = int(center_y // cell_h)

            obj['grid_position'] = (grid_x, grid_y)
            self.grid[grid_y, grid_x] = obj['type']

    # ================== TAMPILKAN HASIL ================== #
    def display_detection_results(self, frame, detected_objects):
        for obj in detected_objects:
            x1,y1,x2,y2 = obj['bbox']
            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
            cv2.putText(frame, obj['class_name'], (x1,y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,0),2)
        return frame

    # untuk pathfinding
    def get_pathfinding_map(self):
        path_map = np.zeros((self.field_height, self.field_width), dtype=int)
        for y in range(self.field_height):
            for x in range(self.field_width):
                if self.grid[y,x] == "Forbidden":
                    path_map[y,x] = 1
        return path_map

    def print_grid_map(self):
        print("\n=== GRID MAP ===")
        for y in range(self.field_height):
            print("|", end=" ")
            for x in range(self.field_width):
                print(f"{self.grid[y,x]:8}", end=" ")
            print("|")
