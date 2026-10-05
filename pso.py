"""
pso.py - Particle Swarm Optimization for grid path planning.

How a path is represented
-------------------------
A particle is NOT a full path. It is a short list of N_WAYPOINTS (x, y)
points with real-valued coordinates. To turn a particle into a path:

    start -> waypoint 1 -> waypoint 2 -> ... -> waypoint N -> goal

every waypoint is rounded to the nearest cell and neighbouring points are
joined with a straight line of grid cells (Bresenham's line algorithm).

Fitness (lower is better)
-------------------------
    fitness = path length + COLLISION_PENALTY * number of collisions

The penalty is bigger than any possible path length, so a path that touches
even one obstacle is always worse than any clean path.
"""

import math

import numpy as np

import config
from grid import is_blocked_move


def line_cells(a, b):
    """Bresenham's line: the grid cells on the straight line from a to b."""
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x1 >= x0 else -1
    sy = 1 if y1 >= y0 else -1
    err = dx - dy
    cells = [(x0, y0)]
    while (x0, y0) != (x1, y1):
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
        cells.append((x0, y0))
    return cells


def decode(position, start, goal, size):
    """Turn a particle position (N_WAYPOINTS x 2 floats) into a list of cells."""
    waypoints = np.clip(np.rint(position), 0, size - 1).astype(int)
    points = [start] + [(int(x), int(y)) for x, y in waypoints] + [goal]

    path = [start]
    for a, b in zip(points[:-1], points[1:]):
        path.extend(line_cells(a, b)[1:])      # [1:] avoids repeating cell a
    return path


def evaluate(path, obstacles):
    """Return (fitness, length, collisions) of a path given as a list of cells."""
    length = 0.0
    collisions = 0
    for a, b in zip(path[:-1], path[1:]):
        length += math.hypot(b[0] - a[0], b[1] - a[1])   # 1 or sqrt(2)
        if is_blocked_move(a, b, obstacles):
            collisions += 1
    fitness = length + config.COLLISION_PENALTY * collisions
    return fitness, length, collisions


def run_pso(obstacles, start, goal, size, seed, verbose=True):
    """Run PSO and return a dictionary with the best path and the history."""
    rng = np.random.default_rng(seed)
    n, k = config.N_PARTICLES, config.N_WAYPOINTS

    # ---- 1. initialise the swarm ----------------------------------------
    # Waypoints start evenly spread on the straight line start -> goal, then
    # get random noise so that every particle tries a different route.
    fractions = np.linspace(0, 1, k + 2)[1:-1].reshape(1, k, 1)
    s, g = np.array(start, dtype=float), np.array(goal, dtype=float)
    straight_line = s + fractions * (g - s)
    positions = straight_line + rng.normal(0, size / 4, (n, k, 2))
    positions = np.clip(positions, 0, size - 1)
    velocities = rng.uniform(-1, 1, (n, k, 2))

    def fitness_of(position):
        return evaluate(decode(position, start, goal, size), obstacles)[0]

    # ---- 2. first evaluation: personal bests and global best -------------
    pbest_pos = positions.copy()
    pbest_fit = np.array([fitness_of(p) for p in positions])
    best = int(np.argmin(pbest_fit))
    gbest_pos = pbest_pos[best].copy()
    gbest_fit = float(pbest_fit[best])

    history = [gbest_fit]
    stale = 0                      # iterations since gbest last improved

    # ---- 3. main loop ----------------------------------------------------
    for it in range(1, config.N_ITERATIONS + 1):
        # inertia goes down linearly: explore first, fine-tune later
        w = config.W_START - (config.W_START - config.W_END) * it / config.N_ITERATIONS

        r1 = rng.random((n, k, 2))
        r2 = rng.random((n, k, 2))

        # velocity update
        velocities = (w * velocities
                      + config.C1 * r1 * (pbest_pos - positions)
                      + config.C2 * r2 * (gbest_pos - positions))
        velocities = np.clip(velocities, -config.V_MAX, config.V_MAX)

        # position update (waypoints must stay inside the grid)
        positions = np.clip(positions + velocities, 0, size - 1)

        # evaluate every particle: decode -> collision check -> fitness
        improved = False
        for i in range(n):
            fit = fitness_of(positions[i])
            if fit < pbest_fit[i]:                 # new personal best
                pbest_fit[i] = fit
                pbest_pos[i] = positions[i].copy()
                if fit < gbest_fit:                # new global best
                    gbest_fit = float(fit)
                    gbest_pos = positions[i].copy()
                    improved = True

        history.append(gbest_fit)
        stale = 0 if improved else stale + 1

        if verbose and (it % 25 == 0 or it == 1):
            print(f"  iteration {it:3d} | inertia {w:.2f} | best fitness {gbest_fit:8.3f}")

        # stopping condition 2: clean path found and no progress for a while
        if stale >= config.PATIENCE and gbest_fit < config.COLLISION_PENALTY:
            if verbose:
                print(f"  stopped early at iteration {it} "
                      f"(no improvement for {config.PATIENCE} iterations)")
            break

    # ---- 4. output -------------------------------------------------------
    path = decode(gbest_pos, start, goal, size)
    fitness, length, collisions = evaluate(path, obstacles)
    waypoints = [(int(x), int(y))
                 for x, y in np.clip(np.rint(gbest_pos), 0, size - 1).astype(int)]
    return {
        "path": path,
        "waypoints": waypoints,
        "length": length,
        "collisions": collisions,
        "fitness": fitness,
        "history": history,
        "iterations": len(history) - 1,
    }
