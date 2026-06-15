import math
import heapq

def is_collision(x, y, obstacles, margin=0.3):
    """Sprawdza czy dany punkt znajduje się wewnątrz przeszkody."""
    if not obstacles: 
        return False
        
    for obs in obstacles:
        ox, oy, w, h = obs['x'], obs['y'], obs['width'], obs['height']
        if (ox - margin <= x <= ox + w + margin) and (oy - margin <= y <= oy + h + margin):
            return True
    return False

def a_star(start, goal, obstacles, step_size=0.5):
    """Oblicza bezkolizyjną ścieżkę algorytmem A*."""
    def snap(val): 
        return round(val / step_size) * step_size
        
    def heuristic(a, b): 
        return math.hypot(a[0]-b[0], a[1]-b[1])

    start_snapped = (snap(start[0]), snap(start[1]))
    goal_snapped = (snap(goal[0]), snap(goal[1]))
    
    open_set = []
    heapq.heappush(open_set, (0, start_snapped))
    came_from = {}
    g_score = {start_snapped: 0}
    
    directions = [
        (step_size, 0), (-step_size, 0), (0, step_size), (0, -step_size),
        (step_size, step_size), (-step_size, step_size), (step_size, -step_size), (-step_size, -step_size)
    ]
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if heuristic(current, goal) < step_size * 1.5:
            path = [list(goal)]
            while current in came_from:
                path.append(list(current))
                current = came_from[current]
            path.reverse()
            return path
            
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            
            if not (-2.0 <= neighbor[0] <= 10.0 and -2.0 <= neighbor[1] <= 10.0):
                continue
            if is_collision(neighbor[0], neighbor[1], obstacles):
                continue
            
            tentative_g = g_score[current] + math.hypot(dx, dy)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal_snapped)
                heapq.heappush(open_set, (f_score, neighbor))
                
    print("Ostrzeżenie: Nie znaleziono bezkolizyjnej ścieżki!")
    return [list(goal)]