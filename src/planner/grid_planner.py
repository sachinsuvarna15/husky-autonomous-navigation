import numpy as np
import heapq
import math


class OccupancyGridMap:
    def __init__(self, map_size=14.0, resolution=0.1):
        self.map_size = map_size
        self.resolution = resolution
        self.grid_size = int(map_size / resolution)

        # Final planning grid (inflated)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)

        # Original obstacle-only grid
        self.original_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)

        # Inflated grid for visualization/planning
        self.inflated_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)

        self.origin = -map_size / 2.0

    def world_to_grid(self, x, y):
        gx = int((x - self.origin) / self.resolution)
        gy = int((y - self.origin) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.resolution + self.origin + self.resolution / 2.0
        y = gy * self.resolution + self.origin + self.resolution / 2.0
        return x, y

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.grid_size and 0 <= gy < self.grid_size

    def is_free(self, gx, gy):
        return self.in_bounds(gx, gy) and self.grid[gy, gx] == 0

    def mark_obstacle_rect(self, position, half_extents):
        x, y, _ = position
        hx, hy, _ = half_extents

        min_x = x - hx
        max_x = x + hx
        min_y = y - hy
        max_y = y + hy

        min_gx, min_gy = self.world_to_grid(min_x, min_y)
        max_gx, max_gy = self.world_to_grid(max_x, max_y)

        for gx in range(min_gx, max_gx + 1):
            for gy in range(min_gy, max_gy + 1):
                if self.in_bounds(gx, gy):
                    self.original_grid[gy, gx] = 1

    def inflate_obstacles(self, inflation_radius=0.35):
        inflation_cells = int(math.ceil(inflation_radius / self.resolution))

        self.inflated_grid = self.original_grid.copy()
        obstacle_indices = np.argwhere(self.original_grid == 1)

        for gy, gx in obstacle_indices:
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    nx = gx + dx
                    ny = gy + dy

                    if not self.in_bounds(nx, ny):
                        continue

                    dist = math.sqrt(dx * dx + dy * dy) * self.resolution
                    if dist <= inflation_radius:
                        self.inflated_grid[ny, nx] = 1

        # Planner uses inflated grid
        self.grid = self.inflated_grid.copy()

    def build_from_obstacles(self, obstacles, inflation_radius=0.45):
        self.grid.fill(0)
        self.original_grid.fill(0)
        self.inflated_grid.fill(0)

        for obs in obstacles:
            self.mark_obstacle_rect(obs["position"], obs["half_extents"])

        self.inflate_obstacles(inflation_radius=inflation_radius)

    def obstacle_proximity_cost(self, gx, gy, radius_cells=4):
        """
        Extra cost for cells close to obstacles.
        This makes A* prefer the middle of open corridors.
        """
        if not self.in_bounds(gx, gy):
            return 1000.0

        cost = 0.0

        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                nx = gx + dx
                ny = gy + dy

                if not self.in_bounds(nx, ny):
                    continue

                if self.grid[ny, nx] == 1:
                    dist = math.sqrt(dx * dx + dy * dy)
                    if dist == 0:
                        return 1000.0
                    cost += 1.0 / dist

        return cost

    def print_summary(self):
        original_count = int(np.sum(self.original_grid == 1))
        inflated_count = int(np.sum(self.inflated_grid == 1))
        free_count = int(np.sum(self.grid == 0))

        print("Grid size:", self.grid.shape)
        print("Original obstacle cells:", original_count)
        print("Inflated obstacle cells:", inflated_count)
        print("Free cells:", free_count)


class AStarPlanner:
    def __init__(self, grid_map):
        self.grid_map = grid_map

        # 8-connected neighbors
        self.neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, 1.414),
            (-1, 1, 1.414),
            (1, -1, 1.414),
            (1, 1, 1.414),
        ]

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def simplify_path(self, path, step=3):
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        for i in range(step, len(path) - 1, step):
            simplified.append(path[i])
        simplified.append(path[-1])
        return simplified

    def plan(self, start, goal):
        if not self.grid_map.is_free(start[0], start[1]):
            print("Start is invalid or blocked")
            return []

        if not self.grid_map.is_free(goal[0], goal[1]):
            print("Goal is invalid or blocked")
            return []

        open_heap = []
        heapq.heappush(open_heap, (0, start))

        came_from = {}
        g_cost = {start: 0.0}
        visited = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                raw_path = self.reconstruct_path(came_from, current)
                return self.simplify_path(raw_path, step=2)

            for dx, dy, move_cost in self.neighbors:
                nx = current[0] + dx
                ny = current[1] + dy
                neighbor = (nx, ny)

                if not self.grid_map.is_free(nx, ny):
                    continue

                # Prevent diagonal corner cutting
                if dx != 0 and dy != 0:
                	side1_blocked = not self.grid_map.is_free(current[0] + dx, current[1])
                	side2_blocked = not self.grid_map.is_free(current[0], current[1] + dy)
                	if side1_blocked and side2_blocked:
                		continue

                proximity_penalty = 0.18 * self.grid_map.obstacle_proximity_cost(
                    nx, ny, radius_cells=2
                )

                tentative_g = g_cost[current] + move_cost + proximity_penalty

                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f, neighbor))

        print("No path found")
        return []

    def path_to_world(self, path):
        world_path = []
        for gx, gy in path:
            wx, wy = self.grid_map.grid_to_world(gx, gy)
            world_path.append((wx, wy))
        return world_path
