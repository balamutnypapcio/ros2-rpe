import py_trees
from behaviors import MoveTo, PickObject, PlaceObject

def create_behavior_tree(config, robot_state):
    """Zwraca gotowe drzewo zachowań na podstawie konfiguracji."""
    root = py_trees.composites.Sequence("Root", memory=True)
    obstacles = config.get("obstacles", [])

    for obj in config["objects"]:
        root.add_children([
            MoveTo(f"MoveTo_{obj['name']}_pickup", obj["position"], robot_state, obstacles),
            PickObject(f"Pick_{obj['name']}", obj["name"], robot_state),
            MoveTo(f"MoveTo_{obj['name']}_drop", obj["drop_position"], robot_state, obstacles),
            PlaceObject(f"Place_{obj['name']}", obj['name'], robot_state),
        ])

    root.add_children([
        MoveTo("MoveTo_start", config["start_pose"], robot_state, obstacles)
    ])

    return root