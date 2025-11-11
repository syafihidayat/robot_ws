import cv2
import numpy as np
from ultralytics import YOLO
import random


class ObjectDetectionMapp:
    def __init__(self,field_size=3):
        self.field_size = field_size
        self.grid = np.full((field_size, field_size), 'Empty', dtype=object)
        self.detected_objects = []
        self.robot_position = (0,0)

        self.model = YOLO('yolo.pt')

        self.target_classes = {
            'real' : 'Target',
            'fake' : 'Forbiden',
            'symbol' : 'Forbiden'
        }

    def generate_synthetic_image(self):

        width, height = 640, 480
        image = np.ones((height,width,3), dtype=np.uint8) * 255

        objects_to_draw = [
            {'class' : 'fake' , 'count' : 3},
            {'class' : 'real' , 'count' : 3},
            {'class' : 'symbol' , 'count' : 2}
        ]

        all_objects = []
        for obj_type in objects_to_draw:
            for i in range(obj_type['count']):
                all_objects.append(obj_type['class'])

        random.shuffle(all_objects)

        position = [
            (100, 100), (300, 100), (500, 100),(700, 100),
            (100, 300), (300, 300), (500, 300),(700, 300),
            (100, 500), (300, 500), (500, 500), (700, 500)
        ]

        for i,(class_name, pos) in enumerate(zip(all_objects, positions)):
            if class_name == 'real':
                color = (0,255 , 0)
                cv2.rectangle(image,(pos[0]-20, pos[1]-40), (pos[0]+20, pos[1]+ 40), color, -1)
                cv2.putText(image, 'KFS REAL', (pos[0]-25, pos[1]-50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,0), 2)

            elif class_name == 'fake':
                color = (0, 0, 255)
                cv2.rectangle(image, (pos[0]-25, pos[1]-30), (pos[0] + 25,pos[1]+ 30), color, -1)
                cv2.putText(image, "KFS FAKE", (pos[0]-15, pos[1]-40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)

            elif class_name == 'symbol' :
                color = (255, 0 , 0)
                cv2.rectangle(image, (pos[0]-30, pos[1]- 50), (pos[0]+30, pos[1]+50),color, -1)
                cv2.putText(image, 'KFS R1', (pos[0]-25, pos[1]-60),
                    cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,255,255), 2)


        return image

    def capture_and_detect(self):

        print("_______CAPTURE AND DETECT ONCE________")

        frame = self.generate_synthetic_image()

        cv2.imwrite('capture_frame_4x3.jpg', frame)

        results = self.model(frame)

        detected_objects = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                class_name = self.model.name[class_id]
                confidence = box.conf[0]

                if class_name in self.target_classes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                    detected_objects.append({
                        'class_name' : class_name,
                        'confidence' : float(confidence),
                        'bbox' : (x1, y1, x2, y2),
                        'type' : self.target_classes[class_name]
                    })

        print(f"Total object terdeteksi: {len(detected_objects)}")
        return frame, detected_objects
        

    def map_to_grid(self, detected_objects, frame_shape):
        """_______Memetakan object yg terditeksi ke grid 4x3___________"""

        height, width = frame_shape[:2]
        grid_cell_width = width / self.field_width
        grid_cell_height = height / self.field_height

        self.grid = np.full((self.field_height, self.field_height), 'Empty', dtype=object)
        self.detected_objects = []

        for obj in detected_objects:
            x1, y1, x2, y2 = obj['bbox']

            center_x = (x1 + x2) / 2
            center_x = (y1 + y2) / 2

            grid_x  = int(center_x / grid_cell_width)
            grid_y  = int(center_y / grid_cell_width)

            grid_x = max(0, min(self.field_width-1, grid_x))
            grid_y = max(0, min(self.field_height-1, grid_y))

            obj_type = obj['type']
            class_name = obj['class_name']
            self.grid[grid_y, grid_x] = f"{obj_type}_{class_name}"

            self.detected_objects.append({
                'grid_position' : (grid_x, grid_y),
                'type' : obj_type,
                'class_name' : class_name,
                'original_bbox' : obj['bbox']
            })

        self.grid[self.robot_position[1], self.robot_position[0]] = 'Robot'

    
    def display_detection_results(self, frame, detected_objects):

        display_frame = frame.copy()
        height, width = frame.shape[:2]
        grid_cell_width = width / self.field_width
        grid_cell_height = height / self.field_height

        for i in range(1, self.field_width):
            cv2.line(display_frame,(int(i * grid_cell_width), 0),
                (int(i * grid_cell_width), height), (0,0,0),2)

        for i in range(1, self.field_height):
            cv2.line(display_frame,(0, int(i * grid_cell_height)),
                (witdh, int(i * grid_cell_height)), (0,0,0),2)

        for obj in detected_objects:
            x1,y1,x2,y2 = map(int, obj['bbox'])
            class_name = obj['class_name']
            obj_type = obj['type']

        if obj_type == 'Target':
            color = (0, 255,0)
        else:
            color = (0 ,0 ,255)

        cv2.rectangle(display_frame,(x1,y1), (x2,y2), color, 3)

        label = f"{class_name} ({obj_type})"
        cv2.putText(display_frame, label,(x1,y1-10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color,2)
        
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        cv2.circle(display_frame,(center_x,center_y), 5,(255,0,0), -1)

        grid_x = int(center_x / grid_cell_width)
        grid_y = int(center_y / grid_cell_height)
        coord_text = f"({grid_x},{grid_y})"
        cv2.putText(display_frame, coord_text, (center_x+10, center_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        robot_x = int((self.robot_position[0] + 0.5) * grid_cell_width)
        robot_y = int((self.robot_position[1] + 0.5) * grid_cell_height)
        cv2.circle(display_frame,(robot_x, robot_y), 10,(0,0,255), -1)
        cv2.putText(display_frame,"Robot", (robot_x-30, robot_y-15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,0,255), 2)


        return display_frame


    def print_grid_map(self):
        """Menampilkan grid map 4x3 dalam format text"""
        print(f"\n=== GRID MAP {self.field_width}x{self.field_height} ===")
        print("Legend: 🤖=Robot, 🎯=Target, ❌=Forbidden, ◻️=Empty")
        print("=" * (self.field_width * 4 + 1))
        
        for i in range(self.field_height):
            row = "|"
            for j in range(self.field_width):
                cell = self.grid[i, j]
                if cell == 'Robot':
                    row += " 🤖 |"
                elif 'Target' in str(cell):
                    row += " 🎯 |"
                elif 'Forbidden' in str(cell):
                    row += " ❌ |"
                else:
                    row += " ◻️ |"
            print(row)
        print("=" * (self.field_width * 4 + 1))

    def get_pathfinding_map(self):
        """Mengembalikan array untuk pathfinding (0=available, 1=obstacle)"""
        pathfinding_grid = np.zeros((self.field_height, self.field_width), dtype=int)
        
        for i in range(self.field_height):
            for j in range(self.field_width):
                cell = self.grid[i, j]
                if 'Forbidden' in str(cell):
                    pathfinding_grid[i, j] = 1  # Obstacle
                else:
                    pathfinding_grid[i, j] = 0  # Available
        
        return pathfinding_grid

class PathFinder:
    def __init__(self, grid_map):
        self.grid = grid_map
        self.height = len(grid_map)
        self.width = len(grid_map[0])
    
    def bfs_find_path(self, start, goal):
        """Mencari path menggunakan BFS algorithm untuk grid 4x3"""
        if (start[0] < 0 or start[0] >= self.width or 
            start[1] < 0 or start[1] >= self.height or
            goal[0] < 0 or goal[0] >= self.width or 
            goal[1] < 0 or goal[1] >= self.height):
            return None
            
        if self.grid[start[1]][start[0]] == 1 or self.grid[goal[1]][goal[0]] == 1:
            return None
        
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Atas, Kanan, Bawah, Kiri
        queue = deque()
        queue.append((start, [start]))
        visited = set([start])
        
        while queue:
            (x, y), path = queue.popleft()
            
            if (x, y) == goal:
                return path
            
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                
                if (0 <= nx < self.width and 0 <= ny < self.height and 
                    self.grid[ny][nx] == 0 and (nx, ny) not in visited):
                    
                    visited.add((nx, ny))
                    queue.append(((nx, ny), path + [(nx, ny)]))
        
        return None
    

    def find_all_target_paths(self, start_position, detected_objects):
        """Mencari path ke semua target objects"""
        paths = {}
        
        for obj in detected_objects:
            if obj['type'] == 'Target':
                target_pos = obj['grid_position']
                path = self.bfs_find_path(start_position, target_pos)
                
                if path:
                    paths[obj['class_name']] = {
                        'position': target_pos,
                        'path': path,
                        'distance': len(path) - 1,
                        'object_type': obj['class_name']
                    }
        
        # Urutkan berdasarkan jarak terdekat
        sorted_paths = dict(sorted(paths.items(), key=lambda x: x[1]['distance']))
        return sorted_paths
    
    def find_optimal_path_sequence(self, start_position, detected_objects):
        """Mencari urutan optimal untuk mengumpulkan semua target"""
        targets = [obj for obj in detected_objects if obj['type'] == 'Target']
        
        if not targets:
            return None
            
        current_pos = start_position
        sequence = []
        total_path = []
        remaining_targets = targets.copy()
        
        while remaining_targets:
            # Cari target terdekat dari posisi sekarang
            closest_target = None
            shortest_path = None
            min_distance = float('inf')
            
            for target in remaining_targets:
                path = self.bfs_find_path(current_pos, target['grid_position'])
                if path and len(path) < min_distance:
                    min_distance = len(path)
                    closest_target = target
                    shortest_path = path
            
            if closest_target and shortest_path:
                sequence.append({
                    'target': closest_target['class_name'],
                    'position': closest_target['grid_position'],
                    'path': shortest_path,
                    'distance': len(shortest_path) - 1
                })
                total_path.extend(shortest_path[:-1])  # Exclude duplicate positions
                current_pos = closest_target['grid_position']
                remaining_targets.remove(closest_target)
            else:
                break
        
        return sequence



