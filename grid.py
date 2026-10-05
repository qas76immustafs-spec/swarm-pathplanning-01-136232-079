"""
grid.py - builds the problem instance from the seed.

Scenario: a rescue robot has to cross a collapsed warehouse floor.
The floor is a square grid, the debris piles are blocked cells, the robot
enters at START and has to reach the survivor at GOAL.

Coordinates are (x, y): x is the column, y is the row.
"""

import heapq
import math
import random


def generate_problem(seed, size, obstacle_ratio, min_dist):
    """Return (obstacles, start, goal), all decided by the seed.

    random.seed(seed) is called once. If a layout is unusable (start and goal
    too close, or goal walled off) the next layout is drawn from the SAME
    random stream, so the result still depends only on the seed.
    """
    random.seed(seed)
    all_cells = [(x, y) for x in range(size) for y in range(size)]
    n_obstacles = int(obstacle_ratio * size * size)

    while True:
        obstacles = set(random.sample(all_cells, n_obstacles))
        free_cells = [c for c in all_cells if c not in obstacles]
        start, goal = random.sample(free_cells, 2)

        far_enough = math.dist(start, goal) >= min_dist
        if far_enough and shortest_path_cost(obstacles, start, goal, size) is not None:
            return obstacles, start, goal


def is_blocked_move(a, b, obstacles):
    """True if the single step a -> b (neighbouring cells) is not allowed.

    A step is blocked when b is an obstacle, or when it is a diagonal step
    that squeezes through the corner between two obstacles.
    """
    if b in obstacles:
        return True
    ax, ay = a
    bx, by = b
    if ax != bx and ay != by:                       # diagonal step
        if (ax, by) in obstacles and (bx, ay) in obstacles:
            return True
    return False


def shortest_path_cost(obstacles, start, goal, size):
    """Dijkstra on the 8-connected grid.

    Used for two things only:
      1. to make sure the generated problem can actually be solved
      2. as a reference value to judge how good the PSO answer is
    Returns the optimal cost, or None if the goal cannot be reached.
    """
    moves = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    best = {start: 0.0}
    queue = [(0.0, start)]
    while queue:
        cost, cell = heapq.heappop(queue)
        if cell == goal:
            return cost
        if cost > best[cell]:
            continue
        for dx, dy in moves:
            nxt = (cell[0] + dx, cell[1] + dy)
            if not (0 <= nxt[0] < size and 0 <= nxt[1] < size):
                continue
            if is_blocked_move(cell, nxt, obstacles):
                continue
            new_cost = cost + math.hypot(dx, dy)
            if new_cost < best.get(nxt, float("inf")):
                best[nxt] = new_cost
                heapq.heappush(queue, (new_cost, nxt))
    return None
