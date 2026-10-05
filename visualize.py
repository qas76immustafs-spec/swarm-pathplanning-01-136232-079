"""
visualize.py - matplotlib drawings of the grid, the path and the convergence.
"""

import matplotlib
matplotlib.use("Agg")              # save figures to files, no window needed
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import config

OBSTACLE_COLOR = "#2f3640"
PATH_COLOR = "#e67e22"
START_COLOR = "#27ae60"
GOAL_COLOR = "#c0392b"


def plot_grid(obstacles, start, goal, size, title, filename,
              path=None, waypoints=None):
    """Draw the map. If a path is given it is drawn on top."""
    fig, ax = plt.subplots(figsize=(7, 7))

    for (x, y) in obstacles:
        ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, color=OBSTACLE_COLOR))

    if path is not None:
        xs = [c[0] for c in path]
        ys = [c[1] for c in path]
        ax.plot(xs, ys, "-o", color=PATH_COLOR, linewidth=2.5, markersize=4,
                label="PSO best path", zorder=3)
    if waypoints is not None:
        ax.scatter([w[0] for w in waypoints], [w[1] for w in waypoints],
                   s=130, facecolors="none", edgecolors="#2980b9", linewidths=2,
                   label="Waypoints (gbest)", zorder=4)

    ax.scatter(*start, s=260, marker="s", color=START_COLOR, label="Start", zorder=5)
    ax.scatter(*goal, s=330, marker="*", color=GOAL_COLOR, label="Goal", zorder=5)
    ax.scatter([], [], s=120, marker="s", color=OBSTACLE_COLOR, label="Obstacle")

    ax.set_xlim(-0.5, size - 0.5)
    ax.set_ylim(-0.5, size - 0.5)
    ax.set_xticks(range(size))
    ax.set_yticks(range(size))
    ax.set_xticks([v - 0.5 for v in range(size + 1)], minor=True)
    ax.set_yticks([v - 0.5 for v in range(size + 1)], minor=True)
    ax.grid(which="minor", color="#dcdde1", linewidth=0.8)
    ax.tick_params(which="minor", length=0)
    ax.tick_params(labelsize=8)
    ax.set_aspect("equal")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title, fontsize=12)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=5,
              frameon=False, fontsize=9)

    fig.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_convergence(history, filename):
    """Best fitness of the swarm after every iteration."""
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(range(len(history)), history, color=PATH_COLOR, linewidth=2)
    ax.axhline(config.COLLISION_PENALTY, color="grey", linestyle="--", linewidth=1)
    ax.text(len(history) - 1, config.COLLISION_PENALTY * 1.08,
            "below this line = no collisions", ha="right", fontsize=8, color="grey")
    ax.set_yscale("log")
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Global best fitness (log scale)")
    ax.set_title("PSO convergence")
    ax.grid(alpha=0.3)
    fig.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close(fig)
