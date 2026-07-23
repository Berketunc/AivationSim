"""Classical grid-based flow-field controller — the "classical" half of the
residual RL architecture (see MILESTONE2_STATUS.md's "Next steps" list,
item 1).

Mirrors oa_planning's real A*+replanning behaviour (astar.py,
planner_params.yaml: 0.2m resolution, 0.6m inflation) but precomputed once as
a static goal-directed vector field rather than searched fresh every control
cycle: since this v1 task's warehouse geometry and goal are fixed (no domain
randomization yet), a single-source Dijkstra from the goal cell over an
inflated occupancy grid gives, for every free cell, the direction of the
shortest obstacle-avoiding path to the goal — exactly what repeated
from-scratch A* replanning converges to at whatever cell the vehicle
currently occupies, just computed once instead of on every ~1s replan cycle.
`WarehouseAvoidanceEnv` adds the learned residual on top of this field's
proposed velocity (see its `_pre_physics_step`).
"""

from __future__ import annotations

import heapq
import math

import numpy as np
import torch

# 8-connected grid neighbors — the 2D analogue of astar.py's 26-connected
# NEIGHBORS_26, since this controller only ever needs xy (see
# warehouse_avoidance_env_cfg's CRUISE_ALTITUDE_Z comment for why z is out of
# scope for this whole task).
_NEIGHBORS_8 = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)]

# Same resolution/inflation oa_planning's real A* planner uses
# (planner_params.yaml) — keeps this classical baseline a fair analogue of
# the real controller rather than an arbitrarily different one.
GRID_RESOLUTION = 0.2
INFLATION_RADIUS_M = 0.6


class GoalFlowField:
    """Precomputed goal-directed unit-velocity field over a 2D grid, routed
    around inflated obstacles via Dijkstra-from-goal. Built once at env
    construction (host CPU, numpy); queried every step via nearest-cell
    lookup (cheap GPU/CPU tensor gather).
    """

    def __init__(
        self,
        grid_origin_xy: tuple[float, float],
        grid_size_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        pillar_xy: list[tuple[float, float]],
        pillar_half_xy: tuple[float, float],
        wall_xy: list[tuple[float, float]],
        wall_half_xy: list[tuple[float, float]],
        device: str,
        resolution: float = GRID_RESOLUTION,
        inflation: float = INFLATION_RADIUS_M,
    ):
        origin_x, origin_y = grid_origin_xy
        self._origin_x = float(origin_x)
        self._origin_y = float(origin_y)
        self._resolution = float(resolution)

        nx = int(round(grid_size_xy[0] / resolution)) + 1
        ny = int(round(grid_size_xy[1] / resolution)) + 1
        self._nx, self._ny = nx, ny

        xs = origin_x + resolution * np.arange(nx)
        ys = origin_y + resolution * np.arange(ny)
        gx, gy = np.meshgrid(xs, ys, indexing="ij")  # (nx, ny)

        occupied = np.zeros((nx, ny), dtype=bool)
        obstacles = [(p, pillar_half_xy) for p in pillar_xy] + list(zip(wall_xy, wall_half_xy))
        for (ox, oy), (hx, hy) in obstacles:
            dx = np.abs(gx - ox) - hx - inflation
            dy = np.abs(gy - oy) - hy - inflation
            occupied |= (dx < 0) & (dy < 0)

        goal_i = int(round((goal_xy[0] - origin_x) / resolution))
        goal_j = int(round((goal_xy[1] - origin_y) / resolution))
        goal_i = min(max(goal_i, 0), nx - 1)
        goal_j = min(max(goal_j, 0), ny - 1)

        cost = self._dijkstra_cost_to_go(occupied, goal_i, goal_j, resolution)

        direction = self._steepest_descent_directions(cost, occupied, resolution)
        self._direction = torch.tensor(direction, dtype=torch.float32, device=device)  # (nx, ny, 2)

    @staticmethod
    def _dijkstra_cost_to_go(occupied: np.ndarray, goal_i: int, goal_j: int, resolution: float) -> np.ndarray:
        nx, ny = occupied.shape
        cost = np.full((nx, ny), np.inf, dtype=np.float64)
        # Goal cell is exempt from the occupied check by construction (see
        # planner_params.yaml's own comment on why spawn/goal need real wall
        # clearance) — start the search there regardless.
        cost[goal_i, goal_j] = 0.0
        visited = np.zeros((nx, ny), dtype=bool)
        heap = [(0.0, goal_i, goal_j)]
        while heap:
            d, i, j = heapq.heappop(heap)
            if visited[i, j]:
                continue
            visited[i, j] = True
            for di, dj in _NEIGHBORS_8:
                ni, nj = i + di, j + dj
                if not (0 <= ni < nx and 0 <= nj < ny) or occupied[ni, nj] or visited[ni, nj]:
                    continue
                step = resolution * math.sqrt(di * di + dj * dj)
                nd = d + step
                if nd < cost[ni, nj]:
                    cost[ni, nj] = nd
                    heapq.heappush(heap, (nd, ni, nj))
        return cost

    @staticmethod
    def _steepest_descent_directions(cost: np.ndarray, occupied: np.ndarray, resolution: float) -> np.ndarray:
        """Unit vector field pointing toward decreasing cost-to-go (central
        differences), i.e. toward the goal along the shortest obstacle-free
        route. Unreachable cells (cost stays inf — fully enclosed pockets,
        if any) get a large-but-finite fill value so the gradient still
        points sensibly toward whatever's reachable nearby rather than
        propagating NaN/inf into the field."""
        finite = cost[np.isfinite(cost)]
        fill = (finite.max() if finite.size else 0.0) + cost.shape[0] + cost.shape[1]
        cost_filled = np.where(np.isfinite(cost), cost, fill)

        nx, ny = cost.shape
        grad_x = np.zeros((nx, ny))
        grad_y = np.zeros((nx, ny))
        grad_x[1:-1, :] = (cost_filled[2:, :] - cost_filled[:-2, :]) / (2 * resolution)
        grad_x[0, :] = (cost_filled[1, :] - cost_filled[0, :]) / resolution
        grad_x[-1, :] = (cost_filled[-1, :] - cost_filled[-2, :]) / resolution
        grad_y[:, 1:-1] = (cost_filled[:, 2:] - cost_filled[:, :-2]) / (2 * resolution)
        grad_y[:, 0] = (cost_filled[:, 1] - cost_filled[:, 0]) / resolution
        grad_y[:, -1] = (cost_filled[:, -1] - cost_filled[:, -2]) / resolution

        direction = np.stack([-grad_x, -grad_y], axis=-1)
        norm = np.linalg.norm(direction, axis=-1, keepdims=True)
        direction = np.divide(direction, norm, out=np.zeros_like(direction), where=norm > 1e-6)
        return direction

    def direction_at(self, pos_xy: torch.Tensor) -> torch.Tensor:
        """Nearest-cell lookup of the precomputed unit flow direction for
        each (N,2) position. Nearest-cell (not bilinear) is deliberate:
        bilinearly blending directions across a wall/pillar cell boundary
        can average a toward-goal direction with an escape-the-obstacle
        direction and produce something meaningless right at tight gaps."""
        idx_x = ((pos_xy[:, 0] - self._origin_x) / self._resolution).round().long().clamp(0, self._nx - 1)
        idx_y = ((pos_xy[:, 1] - self._origin_y) / self._resolution).round().long().clamp(0, self._ny - 1)
        return self._direction[idx_x, idx_y]
