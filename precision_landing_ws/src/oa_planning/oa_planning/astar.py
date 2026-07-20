import heapq
import math

NEIGHBORS_26 = [
    (dx, dy, dz)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
]


def plan(start, goal, grid):
    """3D A* over grid voxels (26-connected). start/goal are grid indices
    (see OccupancyGrid3D.world_to_index). Returns a list of grid indices from
    start to goal inclusive, or None if no path exists.
    """
    if not grid.in_bounds(start) or not grid.in_bounds(goal):
        return None
    if grid.is_occupied(start) or grid.is_occupied(goal):
        return None
    if start == goal:
        return [start]

    open_heap = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        closed.add(current)

        for d in NEIGHBORS_26:
            neighbor = (current[0] + d[0], current[1] + d[1], current[2] + d[2])
            if neighbor in closed or not grid.in_bounds(neighbor) or grid.is_occupied(neighbor):
                continue

            step_cost = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + math.dist(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    return None
