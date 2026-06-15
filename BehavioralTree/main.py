import yaml
import py_trees
import matplotlib.pyplot as plt

from tree_builder import create_behavior_tree
from visualization import draw_scene

def main():
    # 1. Wczytanie konfiguracji
    with open("params.yaml") as f:
        config = yaml.safe_load(f)

    # 2. Inicjalizacja stanu
    robot_state = {
        "position": list(config["start_pose"]),
        "carried_object": None,
        "placed_objects": {}
    }

    # 3. Zbudowanie i konfiguracja drzewa
    root = create_behavior_tree(config, robot_state)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)
    print(py_trees.display.ascii_tree(tree.root))

    # 4. Przygotowanie wizualizacji
    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))

    # 5. Pętla główna (tick)
    for i in range(2000):
        tree.tick()
        draw_scene(ax, config, robot_state)
        plt.pause(0.02)

        if tree.root.status == py_trees.common.Status.SUCCESS:
            print("Zadanie zakończone!")
            break

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()