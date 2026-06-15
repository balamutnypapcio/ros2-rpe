import matplotlib.patches as patches

def draw_scene(ax, config, robot_state):
    """Czyści osie i rysuje aktualny stan sceny."""
    ax.clear()

    # Rysowanie przeszkód
    for obs in config.get("obstacles", []):
        rect = patches.Rectangle((obs['x'], obs['y']), obs['width'], obs['height'], 
                                 linewidth=1, edgecolor='black', facecolor='gray', alpha=0.5)
        ax.add_patch(rect)
        ax.text(obs['x'] + obs['width']/2, obs['y'] + obs['height']/2, obs.get('name', ''), 
                ha="center", va="center", fontsize=8)

    # Rysowanie obiektów
    for obj in config["objects"]:
        if robot_state["carried_object"] == obj["name"]:
            continue
        if robot_state["placed_objects"].get(obj["name"], False):
            pos = obj["drop_position"]
        else:
            pos = obj["position"]
        ax.plot(pos[0], pos[1], "gs", markersize=12)
        ax.text(pos[0], pos[1] + 0.3, obj["name"], ha="center")

    # Miejsca odkładania
    for obj in config["objects"]:
        dp = obj["drop_position"]
        ax.plot(dp[0], dp[1], "kx", markersize=10)

    # Rysowanie robota
    rx, ry = robot_state["position"]
    ax.plot(rx, ry, "ro", markersize=15)
    label = robot_state["carried_object"] or "empty"
    ax.text(rx, ry + 0.3, f"robot ({label})", ha="center")

    ax.set_xlim(-2, 10)
    ax.set_ylim(-2, 10)
    ax.set_aspect("equal")
    ax.set_title("Pick and Place - A* Pathfinding")
    ax.grid(True)