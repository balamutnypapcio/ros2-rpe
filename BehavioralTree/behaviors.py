import math
import py_trees
from pathfinding import a_star

class MoveTo(py_trees.behaviour.Behaviour):
    def __init__(self, name, target, robot_state, obstacles, step_size=0.5):
        super().__init__(name)
        self.target = target
        self.robot_state = robot_state
        self.obstacles = obstacles
        self.step_size = step_size
        self.path = []

    def initialise(self):
        """Oblicza ścieżkę A* przy wejściu w stan RUNNING."""
        pos = self.robot_state["position"]
        self.path = a_star(pos, self.target, self.obstacles)
        print(f"[{self.name}] Obliczono trasę. Liczba waypointów: {len(self.path)}")

    def update(self):
        if not self.path:
            return py_trees.common.Status.SUCCESS

        next_wp = self.path[0]
        pos = self.robot_state["position"]
        dx = next_wp[0] - pos[0]
        dy = next_wp[1] - pos[1]
        dist = math.hypot(dx, dy)

        if dist < self.step_size:
            self.robot_state["position"] = list(next_wp)
            self.path.pop(0)
            
            if not self.path:
                print(f"[{self.name}] Dojechał do {self.target}")
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING

        ratio = self.step_size / dist
        pos[0] += dx * ratio
        pos[1] += dy * ratio
        return py_trees.common.Status.RUNNING

class PickObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, object_name, robot_state):
        super().__init__(name)
        self.object_name = object_name
        self.robot_state = robot_state

    def update(self):
        self.robot_state["carried_object"] = self.object_name
        print(f"[{self.name}] Podniesiono {self.object_name}")
        return py_trees.common.Status.SUCCESS

class PlaceObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, object_name, robot_state):
        super().__init__(name)
        self.object_name = object_name
        self.robot_state = robot_state

    def update(self):
        self.robot_state["carried_object"] = None
        self.robot_state["placed_objects"][self.object_name] = True
        return py_trees.common.Status.SUCCESS