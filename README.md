# Swarm-Based Path Planning with Obstacles (PSO)

**Swarm Intelligence Lab - Assignment 1**

| | |
|---|---|
| **Name** | Qasim Mustafa |
| **Roll number** | 01-136232-079 |
| **Seed used** | `1136232079` (roll number with the dashes removed) |
| **Algorithm** | Particle Swarm Optimization (PSO) |

## The problem

My scenario is a **rescue robot crossing a collapsed warehouse floor**. The floor is a 20 x 20 grid, the debris piles are blocked cells, and the robot has to get from the entry point (start) to the survivor (goal) by the shortest route without touching any debris.

Nothing is hardcoded. `grid.py` calls `random.seed(1136232079)` and then generates the obstacles, the start and the goal. For my seed this gives:

| Item | Value |
|---|---|
| Grid | 20 x 20 |
| Obstacles | 88 cells (22% of the grid) |
| Start | (18, 19) |
| Goal | (0, 0) |

![Problem instance](results/problem.png)

## My approach

**Particle = a few waypoints, not a whole path.** Each particle holds 6 waypoints, so its position is 12 real numbers. To get a path I round the waypoints to cells and join `start -> wp1 -> ... -> wp6 -> goal` with straight grid lines (Bresenham's line algorithm).

**Fitness (lower is better):**

```
fitness = path length + 100 x number of collisions
```

A straight step costs 1 and a diagonal step costs 1.414. A collision is a step into an obstacle cell, or a diagonal step squeezed between two obstacle cells. No path on this grid can be 100 long, so any path with a collision always loses to a clean path.

**PSO update, every iteration:**

```
v = w*v + c1*r1*(pbest - x) + c2*r2*(gbest - x)
x = x + v
```

| Parameter | Value |
|---|---|
| Particles | 60 |
| Waypoints per particle | 6 |
| Max iterations | 300 |
| Inertia w | 0.9 decreasing linearly to 0.4 |
| c1, c2 | 1.5, 1.5 |
| Max velocity | 4 cells per iteration |
| Early stop | clean path found and no improvement for 80 iterations |

Velocities are clamped to the max velocity and waypoints are clamped to stay inside the grid. The swarm starts with waypoints spread along the straight line from start to goal plus random noise, so the particles begin with different routes.

## Flow diagram (hand-drawn)

The diagram is drawn on two pages. Page 1 goes from Start to decoding a particle into a path, page 2 continues from the collision check to End.

<p align="center">
  <img src="docs/flow_diagram_1.jpg" alt="Hand-drawn flow diagram, page 1" width="48%">
  <img src="docs/flow_diagram_2.jpg" alt="Hand-drawn flow diagram, page 2" width="48%">
</p>

## Results

| Metric | Value |
|---|---|
| Path length (cost) | **29.385** |
| Cells on the path | 25 |
| Collisions | 0 |
| Iterations used | 178 (stopped early) |
| True shortest path (Dijkstra, for reference only) | 28.799 |
| Gap to the optimum | 2.0% |

![Best path](results/best_path.png)

![Convergence](results/convergence.png)

In the first iterations the best fitness is above 100, which means every particle is still hitting obstacles. Once it drops below 100 the swarm has found a collision-free route and from then on it only shortens it.

Dijkstra is not part of the solution. I only use it to check that the generated map is solvable and to see how close PSO gets to the real shortest path.

## How to run

Python 3.9 or newer.

```bash
git clone https://github.com/qas76immustafs-spec/swarm-pathplanning-01-136232-079.git
cd swarm-pathplanning-01-136232-079
pip install -r requirements.txt
python main.py
```

The program prints the problem, the PSO progress and the best path with its cost, and saves three figures in `results/`.

To try another problem instance, change `ROLL_NUMBER` in `config.py`.

## Files

| File | What it does |
|---|---|
| `config.py` | Roll number, seed and all parameters |
| `grid.py` | Generates obstacles, start and goal from the seed |
| `pso.py` | Path decoding, collision check, fitness and the PSO loop |
| `visualize.py` | matplotlib plots |
| `main.py` | Runs everything |
| `results/` | Output figures |
| `docs/flow_diagram_1.jpg`, `docs/flow_diagram_2.jpg` | Photos of my hand-drawn flow diagram (2 pages) |
