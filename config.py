"""
config.py - every setting of the project lives here.

Nothing about the problem itself (obstacles, start, goal) is written by hand:
it is all generated from SEED, which is my roll number.
"""

# ---------------------------------------------------------------- identity
STUDENT_NAME = "Qasim Mustafa"
ROLL_NUMBER = "01-136232-079"

# The seed is the roll number with the dashes removed:
#   "01-136232-079"  ->  "01136232079"  ->  1136232079
SEED = int(ROLL_NUMBER.replace("-", ""))

# ---------------------------------------------------------------- problem
GRID_SIZE = 20            # the map is GRID_SIZE x GRID_SIZE cells
OBSTACLE_RATIO = 0.22     # fraction of cells that are blocked (debris)
MIN_START_GOAL_DIST = 14  # start and goal must be at least this far apart

# ---------------------------------------------------------------- PSO
N_PARTICLES = 60          # swarm size
N_WAYPOINTS = 6           # waypoints per particle (12 numbers per particle)
N_ITERATIONS = 300        # stopping condition 1: maximum iterations
PATIENCE = 80             # stopping condition 2: stop if no improvement

W_START = 0.9             # inertia weight at the first iteration
W_END = 0.4               # inertia weight at the last iteration
C1 = 1.5                  # cognitive coefficient (pull to personal best)
C2 = 1.5                  # social coefficient (pull to global best)
V_MAX = 4.0               # velocity clamp, in cells per iteration

COLLISION_PENALTY = 100.0 # cost added for every blocked cell a path touches
