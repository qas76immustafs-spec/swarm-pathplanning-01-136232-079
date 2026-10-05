"""
main.py - run the whole assignment:  python main.py

Swarm Intelligence Lab - Assignment 1
Swarm-Based Path Planning with Obstacles (PSO)
"""

import os

import config
from grid import generate_problem, shortest_path_cost
from pso import run_pso
from visualize import plot_convergence, plot_grid


def main():
    os.makedirs("results", exist_ok=True)
    size = config.GRID_SIZE

    # 1. problem instance, generated from the roll number
    obstacles, start, goal = generate_problem(
        config.SEED, size, config.OBSTACLE_RATIO, config.MIN_START_GOAL_DIST)

    print("=" * 60)
    print("PSO path planning - rescue robot in a debris field")
    print("=" * 60)
    print(f"Student     : {config.STUDENT_NAME}")
    print(f"Roll number : {config.ROLL_NUMBER}")
    print(f"Seed        : {config.SEED}")
    print(f"Grid        : {size} x {size}")
    print(f"Obstacles   : {len(obstacles)} cells")
    print(f"Start       : {start}")
    print(f"Goal        : {goal}")
    print()

    plot_grid(obstacles, start, goal, size,
              f"Problem instance (seed {config.SEED})", "results/problem.png")

    # 2. PSO
    print("Running PSO ...")
    result = run_pso(obstacles, start, goal, size, config.SEED)

    # 3. report
    optimal = shortest_path_cost(obstacles, start, goal, size)
    print()
    print("Best path found (x, y):")
    print("  " + " -> ".join(str(c) for c in result["path"]))
    print()
    print(f"Waypoints of best particle : {result['waypoints']}")
    print(f"Cells on path              : {len(result['path'])}")
    print(f"Path length (cost)         : {result['length']:.3f}")
    print(f"Collisions                 : {result['collisions']}")
    print(f"Iterations used            : {result['iterations']}")
    print(f"Reference optimum (Dijkstra): {optimal:.3f}")
    print(f"PSO is {100 * (result['length'] / optimal - 1):.1f}% longer than the optimum")

    # 4. figures
    plot_grid(obstacles, start, goal, size,
              f"PSO best path - length {result['length']:.2f}, "
              f"collisions {result['collisions']}",
              "results/best_path.png",
              path=result["path"], waypoints=result["waypoints"])
    plot_convergence(result["history"], "results/convergence.png")
    print()
    print("Figures saved in results/: problem.png, best_path.png, convergence.png")


if __name__ == "__main__":
    main()
