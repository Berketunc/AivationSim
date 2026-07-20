import numpy as np


class OccupancyGrid3D:
    """Bounded 3D voxel occupancy grid, rebuilt each update from a flat list
    of occupied-cell center points (i.e. octomap_server's own
    /octomap_point_cloud_centers output — the occupancy decision already made
    by octomap_server, just handed to us as points instead of an octree).

    is_occupied() checks an inflated copy of the raw occupied set, not the
    raw set itself: A* treats a cell as free the moment it's outside the
    mapped obstacle's voxel, but the vehicle isn't a point — with zero
    margin, a "clear" path can still graze real geometry within the
    airframe's rotor radius. inflation_radius_m should be set to roughly
    that radius. Call refresh_inflation() once per planning cycle (it's too
    expensive to redo on every point-cloud callback) rather than on every
    set_occupied_from_points() call.
    """

    def __init__(self, origin, size, resolution, inflation_radius_m=0.0):
        self.origin = np.array(origin, dtype=float)
        self.resolution = float(resolution)
        self.dims = tuple(max(1, int(round(s / resolution))) for s in size)
        self.raw_occupied = frozenset()
        self.occupied = frozenset()

        radius_cells = max(0, round(inflation_radius_m / self.resolution))
        self._inflation_offsets = self._sphere_offsets(radius_cells)

    @staticmethod
    def _sphere_offsets(radius_cells):
        if radius_cells <= 0:
            return [(0, 0, 0)]
        r = radius_cells
        return [
            (di, dj, dk)
            for di in range(-r, r + 1)
            for dj in range(-r, r + 1)
            for dk in range(-r, r + 1)
            if di * di + dj * dj + dk * dk <= r * r
        ]

    def world_to_index(self, point):
        rel = (np.asarray(point, dtype=float) - self.origin) / self.resolution
        return tuple(int(np.floor(v)) for v in rel)

    def index_to_world(self, index):
        return tuple(
            float(self.origin[i] + (index[i] + 0.5) * self.resolution)
            for i in range(3)
        )

    def in_bounds(self, index):
        return all(0 <= index[i] < self.dims[i] for i in range(3))

    def is_occupied(self, index):
        return index in self.occupied

    def set_occupied_from_points(self, points):
        occ = set()
        for p in points:
            idx = self.world_to_index(p)
            if self.in_bounds(idx):
                occ.add(idx)
        self.raw_occupied = frozenset(occ)

    def refresh_inflation(self):
        """Recompute the inflated occupied set from the current raw one.
        Call this once right before planning, not on every point-cloud
        update — it's O(len(raw_occupied) * len(offsets)).

        Tried vectorizing this with numpy (broadcast-add the offsets, then
        np.unique(axis=0) to dedup) expecting it to be faster for a larger
        radius. It was 60-100x *slower* in practice: np.unique on rows of a
        2D int array does a full lexicographic sort, which loses badly to
        Python's hash-based set for this many small tuples. Left as a plain
        loop deliberately — don't "optimize" this again without benchmarking
        against the real occupied-cell counts this actually sees.
        """
        if self._inflation_offsets == [(0, 0, 0)]:
            self.occupied = self.raw_occupied
            return
        inflated = set()
        for (i, j, k) in self.raw_occupied:
            for (di, dj, dk) in self._inflation_offsets:
                inflated.add((i + di, j + dj, k + dk))
        self.occupied = frozenset(inflated)
