from collections import deque

class PathFinder:
    def __init__(self, grid_map):
        self.grid = grid_map
        self.size = len(grid_map)

    def bfs_find_path(self, start, goal):
        
        if self.grid[start[1]][start[0]] == 1 or self.gird[goal[1]][goal[0]] == 1:
            return None

        directions = [(0,1), (1,0), (0,-1),(-1,0)]
        queue = deque()
        queue.append((start,[start]))
        visited = set([start])

        while queue:
            (x, y), path = queue.popleft()

            if(x, y) == goal:
                return path

            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                if(0 <= nx < self.size and 0 <= ny < self.size and
                    self.grid[ny][nx] == 0 and (nx,ny ) not in visited):

                    visited.add((nx, ny))
                    queue.append(((nx, ny), path + [(nx, ny)]))

        return None

    def find_all_target_paths(self, start_position, target_objects):

        paths = {}

        for target in target_objects:
            if target['type'] == 'Target':
                target_pos = target['grid_position']
                path = self.bfs_find_path(start_position,target_pos)

                if path:
                    paths[target['class_name']] = {
                        'position' : target_pos,
                        'path'     : path,
                        'distance' : len(path) -1
                    }

        return dict(sorted(paths.items(), key=lambda x: x[1]['distance']))

        # return paths

    def find_optimal_path_sequence(self, start_position, detected_objects):
        """Mencari urutan optimal untuk mengumpulkan semua target - GREEDY APPROACH"""
        targets = [obj for obj in detected_objects if obj['type'] == 'Target']
        
        if not targets:
            return None
            
        current_pos = start_position
        sequence = []
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
                current_pos = closest_target['grid_position']
                remaining_targets.remove(closest_target)
            else:
                # Tidak bisa mencapai target yang tersisa
                break
        
        return sequence if sequence else None