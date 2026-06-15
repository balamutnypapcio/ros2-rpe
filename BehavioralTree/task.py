import yaml
import math
import py_trees
import time
import matplotlib.pyplot as plt

class MoveTo(py_trees.behaviour.Behaviour):
    def __init__(self, name, target, robot_state, step_size=0.1):
        super().__init__(name)
        self.target = target
        self.robot_state = robot_state
        self.step_size = step_size

    def update(self):
        pos = self.robot_state["position"]
        dx = self.target[0] - pos[0]
        dy = self.target[1] - pos[1]
        dist = math.hypot(dx, dy)

        if dist < self.step_size:
            self.robot_state["position"] = list(self.target)
            print(f"[{self.name}] Dojechał do {self.target}")
            return py_trees.common.Status.SUCCESS

        # krok w stronę celu
        ratio = self.step_size / dist
        pos[0] += dx * ratio
        pos[1] += dy * ratio
        #print(f"[{self.name}] Jedzie... pozycja: ({pos[0]:.2f}, {pos[1]:.2f})")
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
    

def create_behavior_tree(config, robot_state):
    root = py_trees.composites.Sequence("Root", memory=True)

    for obj in config["objects"]:
        root.add_children([
            MoveTo(f"MoveTo_{obj['name']}_pickup", obj["position"], robot_state),
            PickObject(f"Pick_{obj['name']}", obj["name"], robot_state),
            MoveTo(f"MoveTo_{obj['name']}_drop", obj["drop_position"], robot_state),
            PlaceObject(f"Place_{obj['name']}", obj['name'], robot_state),
        ])

    root.add_children([
        MoveTo("MoveTo_start", config["start_pose"], robot_state)
    ])

    return root


def draw_scene(ax, config, robot_state):
    ax.clear()

    # obiekty (jeśli jeszcze nie podniesione, albo te które zostały odłożone)
    for obj in config["objects"]:
        if robot_state["carried_object"] == obj["name"]:
            continue  # jest przenoszony, nie rysuj na pozycji startowej
        if robot_state["placed_objects"].get(obj["name"], False):
            pos = obj["drop_position"]
        else:
            pos = obj["position"]
        ax.plot(pos[0], pos[1], "gs", markersize=12)
        ax.text(pos[0], pos[1] + 0.3, obj["name"], ha="center")

    # miejsca odkładania (na wszelki wypadek zawsze pokaż jako markery)
    for obj in config["objects"]:
        dp = obj["drop_position"]
        ax.plot(dp[0], dp[1], "kx", markersize=10)

    # robot
    rx, ry = robot_state["position"]
    ax.plot(rx, ry, "ro", markersize=15)
    label = robot_state["carried_object"] or "empty"
    ax.text(rx, ry + 0.3, f"robot ({label})", ha="center")

    ax.set_xlim(-2, 10)
    ax.set_ylim(-2, 10)
    ax.set_aspect("equal")
    ax.set_title("Pick and Place - Behavior Tree")
    ax.grid(True)



if __name__ == "__main__":
    with open("params.yaml") as f:
        config = yaml.safe_load(f)

    robot_state = {
        "position": list(config["start_pose"]),
        "carried_object": None,
        "placed_objects": {}
    }

    root = create_behavior_tree(config, robot_state)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)

    print(py_trees.display.ascii_tree(tree.root))

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))

    for i in range(2000):
        tree.tick()
        draw_scene(ax, config, robot_state)
        plt.pause(0.02)

        if tree.root.status == py_trees.common.Status.SUCCESS:
            print("Zadanie zakończone!")
            break

    plt.ioff()
    plt.show()