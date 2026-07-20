import numpy as np


class OccupancyGrid3D:
    """Bounded 3D voxel occupancy grid, rebuilt each update from a flat list
    of occupied-cell center points (i.e. octomap_server's own
    /octomap_point_cloud_centers output — the occupancy decision already made
    by octomap_server, just handed to us as points instead of an octree).
    """

    def __init__(self, origin, size, resolution):
        self.origin = np.array(origin, dtype=float)
        self.resolution = float(resolution)
        self.dims = tuple(max(1, int(round(s / resolution))) for s in size)
        self.occupied = frozenset()

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
        self.occupied = frozenset(occ)
