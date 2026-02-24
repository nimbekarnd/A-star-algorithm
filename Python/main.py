import os
from astar_class import Astar

OUTPUT_DIR   = os.path.join(os.path.dirname(__file__), "..", "output")
OUTPUT_FILE  = "astar_algoritm_in_python.avi"
OUTPUT_IMAGE = "astar_python_path.png"


def get_input(prompt, cast, valid=None, hint=""):
    while True:
        try:
            val = cast(input(prompt))
            if valid and not valid(val):
                print(f"  Invalid value. {hint}")
                continue
            return val
        except ValueError:
            print(f"  Please enter a valid {cast.__name__}. {hint}")


def main():
    print("=" * 50)
    print("          A* Path Planner")
    print("=" * 50)
    print("Map size: 300 cols x 200 rows\n")

    # Ask for robot parameters first so valid ranges can be shown for start/goal
    print("--- Robot Parameters ---")
    radius    = get_input("  Robot radius       (int, >= 0) : ", int,
                          lambda v: v >= 0)
    clearance = get_input("  Obstacle clearance (int, >= 0) : ", int,
                          lambda v: v >= 0)

    margin  = 1 + radius + clearance
    row_min = float(margin)
    row_max = float(200 - radius - clearance)
    col_min = float(margin)
    col_max = float(300 - radius - clearance)

    if row_min > row_max or col_min > col_max:
        print(f"\n  Error: radius + clearance ({radius + clearance}) is too large for the map.")
        return

    print(f"\n  Valid position range: row [{row_min} – {row_max}], col [{col_min} – {col_max}]")

    astar_tmp = Astar((row_min, col_min, 0), (row_max, col_max), clearance, radius)

    def is_free(row, col):
        return astar_tmp.IsValid(row, col) and not astar_tmp.IsObstacle(row, col)

    print("\n--- Start Position ---")
    while True:
        start_row = get_input(f"  Start row   (float, {row_min} – {row_max}) : ", float,
                              lambda v: row_min <= v <= row_max,
                              f"Must be between {row_min} and {row_max}.")
        start_col = get_input(f"  Start col   (float, {col_min} – {col_max}) : ", float,
                              lambda v: col_min <= v <= col_max,
                              f"Must be between {col_min} and {col_max}.")
        if not is_free(start_row, start_col):
            print("  Start position is inside an obstacle. Please choose another.")
            continue
        break
    start_angle = get_input("  Start angle (int, 0/30/60/.../330)    : ", int,
                            lambda v: 0 <= v < 360 and v % 30 == 0,
                            "Must be a multiple of 30 between 0 and 330.")

    print("\n--- Goal Position ---")
    while True:
        goal_row = get_input(f"  Goal row    (float, {row_min} – {row_max}) : ", float,
                             lambda v: row_min <= v <= row_max,
                             f"Must be between {row_min} and {row_max}.")
        goal_col = get_input(f"  Goal col    (float, {col_min} – {col_max}) : ", float,
                             lambda v: col_min <= v <= col_max,
                             f"Must be between {col_min} and {col_max}.")
        if not is_free(goal_row, goal_col):
            print("  Goal position is inside an obstacle. Please choose another.")
            continue
        break

    start = (start_row, start_col, start_angle)
    goal  = (goal_row, goal_col)

    print("\n" + "=" * 50)
    print(f"  Start : row={start_row}, col={start_col}, angle={start_angle}°")
    print(f"  Goal  : row={goal_row}, col={goal_col}")
    print(f"  Robot : radius={radius}, clearance={clearance}")
    print("=" * 50)
    print("Running A* ...")

    astar = Astar(start, goal, clearance, radius)
    explored, backtrack, cost = astar.Astar()

    if len(backtrack) == 0:
        print("No path found.")
        return

    print(f"\nPath cost     : {cost:.4f}")
    print(f"Path length   : {len(backtrack)} nodes")
    print(f"Nodes explored: {len(explored)}")

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    video_path = os.path.join(OUTPUT_DIR, OUTPUT_FILE)
    image_path = os.path.join(OUTPUT_DIR, OUTPUT_IMAGE)
    print(f"\nGenerating video -> {video_path}")
    astar.animate(explored, backtrack, video_path, image_path)
    print(f"Final image saved -> {image_path}")
    print("Done.")


if __name__ == "__main__":
    main()
