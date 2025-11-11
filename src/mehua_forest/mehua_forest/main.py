import cv2
import numpy as np
from .cam import ObjectDetectionMapper 
from .bfs import PathFinder 

def main():
    # Inisialisasi mapper dengan grid 4x3
    mapper = ObjectDetectionMapper(field_width=4, field_height=3)
    
    print("=== SISTEM DETEKSI OBJEK DAN MAPPING 4x3 ===")
    print("Melakukan capture dan deteksi sekali...")
    
    # Capture dan deteksi sekali
    frame, detected_objects = mapper.capture_and_detect_once()
    
    print(f"\nObject yang terdeteksi:")
    for i, obj in enumerate(detected_objects):
        print(f"{i+1}. {obj['class_name']} ({obj['type']}) - Confidence: {obj['confidence']:.2f}")
    
    # Map ke grid 4x3
    mapper.map_to_grid(detected_objects, frame.shape)
    
    # Tampilkan hasil
    display_frame = mapper.display_detection_results(frame, detected_objects)
    
    # Simpan dan tampilkan gambar hasil deteksi
    cv2.imwrite('detection_results_4x3.jpg', display_frame)
    cv2.imshow('Object Detection Results 4x3', display_frame)
    cv2.waitKey(3000)
    cv2.destroyAllWindows()
    
    # Tampilkan grid map
    mapper.print_grid_map()
    
    # Dapatkan array untuk pathfinding
    pathfinding_map = mapper.get_pathfinding_map()
    print(f"\nArray Pathfinding {mapper.field_width}x{mapper.field_height} (0=available, 1=obstacle):")
    print(pathfinding_map)
    
    # Inisialisasi path finder
    path_finder = PathFinder(pathfinding_map)
    
    # 1. Cari path ke semua target individual
    start_position = mapper.robot_position
    target_paths = path_finder.find_all_target_paths(start_position, mapper.detected_objects)
    
    print(f"\n=== HASIL PATHFINDING INDIVIDUAL ===")
    if target_paths:
        for target_name, path_info in target_paths.items():
            print(f"\nPath ke {target_name}:")
            print(f"Posisi: {path_info['position']}")
            print(f"Jarak: {path_info['distance']} langkah")
            print(f"Jalur: {path_info['path']}")
    else:
        print("Tidak ada path yang ditemukan ke target manapun")
    
    # 2. Cari urutan optimal untuk semua target
    print(f"\n=== URUTAN OPTIMAL PENJEMPUTAN TARGET ===")
    optimal_sequence = path_finder.find_optimal_path_sequence(start_position, mapper.detected_objects)
    
    if optimal_sequence:
        total_steps = 0
        current_pos = start_position
        
        print("Rencana perjalanan optimal:")
        for i, step in enumerate(optimal_sequence):
            print(f"\n{i+1}. Ke {step['target']} di {step['position']}:")
            print(f"   Dari {current_pos} ke {step['position']}")
            print(f"   Jarak: {step['distance']} langkah")
            print(f"   Jalur: {step['path']}")
            
            # Simulasi pergerakan
            print("   Pergerakan:")
            for move_step, pos in enumerate(step['path'][1:], 1):  # Skip start position
                print(f"     Langkah {move_step}: Pindah ke {pos}")
            
            total_steps += step['distance']
            current_pos = step['position']
        
        print(f"\nTotal langkah untuk semua target: {total_steps}")
    else:
        print("Tidak bisa menemukan urutan optimal")
    
    # Tampilkan data lengkap untuk debugging
    print(f"\n=== DATA LENGKAP GRID 4x3 ===")
    print(f"Posisi Robot: {mapper.robot_position}")
    print(f"Dimensi Grid: {mapper.field_width} x {mapper.field_height}")
    print(f"Grid Map Detail:")
    for i in range(mapper.field_height):
        for j in range(mapper.field_width):
            print(f"  [{j},{i}]: {mapper.grid[i,j]}")
    
    print(f"\nObject dalam grid:")
    for obj in mapper.detected_objects:
        print(f"  {obj['class_name']} di {obj['grid_position']} ({obj['type']})")

if __name__ == "__main__":
    main()