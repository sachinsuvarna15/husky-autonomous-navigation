# pylint: disable=no-member, unused-argument, missing-module-docstring,
# missing-class-docstring, missing-function-docstring
import cv2
import numpy as np


class MapGUI:
    def __init__(self, grid_map, planner, window_name="Occupancy Grid"):
        self.grid_map = grid_map
        self.planner = planner
        self.window_name = window_name

        self.scale = 6

        self.start = None
        self.goal = None
        self.path = []
        self.path_world = []
        self.robot_grid = None

        # Pan / zoom
        self.offset_x = 0
        self.offset_y = 0
        self.zoom = 1.0
        self.auto_center = True

        # Canvas
        self.canvas_width = 1200
        self.canvas_height = 1000
        self.base_margin = 100

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

    def image_to_grid(self, x_img, y_img, image_height):
        """
        Convert canvas click coordinates to grid coordinates,
        considering pan, zoom, margin, and vertical flip.
        """
        local_x = x_img - (self.base_margin + self.offset_x)
        local_y = y_img - (self.base_margin + self.offset_y)

        if local_x < 0 or local_y < 0:
            return None, None

        effective_scale = self.scale * self.zoom
        if effective_scale <= 0:
            return None, None

        px = int(local_x / effective_scale)
        py_flipped = int(local_y / effective_scale)

        grid_h = image_height
        grid_w = self.grid_map.grid.shape[1]

        if px < 0 or px >= grid_w or py_flipped < 0 or py_flipped >= grid_h:
            return None, None

        gy = grid_h - 1 - py_flipped
        gx = px

        return gx, gy

    def is_cell_safe(self, gx, gy, clearance_cells=1):
        """
        Reject cells too close to inflated obstacles.
        """
        for dx in range(-clearance_cells, clearance_cells + 1):
            for dy in range(-clearance_cells, clearance_cells + 1):
                nx = gx + dx
                ny = gy + dy

                if not self.grid_map.in_bounds(nx, ny):
                    return False

                if self.grid_map.grid[ny, nx] == 1:
                    return False

        return True

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        image_height = self.grid_map.grid.shape[0]
        gx, gy = self.image_to_grid(x, y, image_height)

        if gx is None or gy is None:
            print("Clicked outside visible map area")
            return

        if not self.grid_map.in_bounds(gx, gy):
            print("Clicked outside map bounds")
            return

        wx, wy = self.grid_map.grid_to_world(gx, gy)

        if self.grid_map.grid[gy, gx] == 1:
            print(f"Clicked obstacle cell: ({gx}, {gy})")
            return

        if not self.is_cell_safe(gx, gy, clearance_cells=1):
            print(f"Clicked too close to obstacle: ({gx}, {gy})")
            return

        if self.start is None:
            self.start = (gx, gy)
            self.goal = None
            self.path = []
            self.path_world = []
            print(f"Start selected: grid={self.start}, world=({wx:.2f}, {wy:.2f})")

        elif self.goal is None:
            if (gx, gy) == self.start:
                print("Goal cannot be same as start")
                return

            self.goal = (gx, gy)
            print(f"Goal selected: grid={self.goal}, world=({wx:.2f}, {wy:.2f})")

            self.path = self.planner.plan(self.start, self.goal)
            self.path_world = self.planner.path_to_world(self.path)

            print(f"Path length: {len(self.path)} cells")

        else:
            self.start = (gx, gy)
            self.goal = None
            self.path = []
            self.path_world = []
            print(f"Reset: new start selected: grid={self.start}, world=({wx:.2f}, {wy:.2f})")

    def update_robot_pose(self, x, y):
        gx, gy = self.grid_map.world_to_grid(x, y)
        if self.grid_map.in_bounds(gx, gy):
            self.robot_grid = (gx, gy)
        else:
            self.robot_grid = None

    def grid_to_image(self):
        """
        Create the base map image before pan/zoom placement.
        Colors:
        - white = free
        - black = original obstacle
        - red = inflated-only area
        - blue = path
        - green = start
        - red circle = goal
        - yellow = robot
        """
        h, w = self.grid_map.grid.shape
        img = np.ones((h, w, 3), dtype=np.uint8) * 255

        # Draw inflated-only area in red
        inflated_only = (self.grid_map.inflated_grid == 1) & (self.grid_map.original_grid == 0)
        img[inflated_only] = [0, 0, 255]

        # Draw original obstacles in black
        original_obstacles = self.grid_map.original_grid == 1
        img[original_obstacles] = [0, 0, 0]

        # Draw path in blue
        for cell in self.path:
            px, py = cell
            if self.grid_map.in_bounds(px, py):
                img[py, px] = [255, 0, 0]

        # Draw start in green
        if self.start is not None:
            sx, sy = self.start
            cv2.circle(img, (sx, sy), 2, (0, 255, 0), -1)

        # Draw goal as red circle
        if self.goal is not None:
            gx, gy = self.goal
            cv2.circle(img, (gx, gy), 2, (0, 0, 255), -1)

        # Draw robot in yellow
        if self.robot_grid is not None:
            rx, ry = self.robot_grid
            cv2.circle(img, (rx, ry), 2, (0, 255, 255), -1)

        # Flip vertically for natural top-view feel
        img = cv2.flip(img, 0)

        # Base scale
        img = cv2.resize(
            img,
            (img.shape[1] * self.scale, img.shape[0] * self.scale),
            interpolation=cv2.INTER_NEAREST
        )

        return img

    def show(self):
        img = self.grid_to_image()

        # Apply zoom
        h, w = img.shape[:2]
        zoomed_w = max(1, int(w * self.zoom))
        zoomed_h = max(1, int(h * self.zoom))
        img = cv2.resize(
            img,
            (zoomed_w, zoomed_h),
            interpolation=cv2.INTER_NEAREST
        )

        h, w = img.shape[:2]

        # Fixed canvas
        canvas = np.ones(
            (self.canvas_height, self.canvas_width, 3),
            dtype=np.uint8
        ) * 255

        # Auto-center map
        if self.auto_center:
            self.offset_x = (self.canvas_width - w) // 2 - self.base_margin
            self.offset_y = (self.canvas_height - h) // 2 - self.base_margin
            self.auto_center = False

        x_offset = self.base_margin + self.offset_x
        y_offset = self.base_margin + self.offset_y

        x1 = max(0, x_offset)
        y1 = max(0, y_offset)
        x2 = min(self.canvas_width, x_offset + w)
        y2 = min(self.canvas_height, y_offset + h)

        if x1 < x2 and y1 < y2:
            src_x1 = x1 - x_offset
            src_y1 = y1 - y_offset
            src_x2 = src_x1 + (x2 - x1)
            src_y2 = src_y1 + (y2 - y1)

            canvas[y1:y2, x1:x2] = img[src_y1:src_y2, src_x1:src_x2]

        cv2.putText(
            canvas,
            "Click: Start/Goal | WASD: Pan | +/-: Zoom | 0: Reset View | ESC: Quit",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (50, 50, 50),
            1,
            cv2.LINE_AA
        )
        cv2.putText(
            canvas,
            f"Zoom: {self.zoom:.2f}  Offset: ({self.offset_x}, {self.offset_y})",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (50, 50, 50),
            1,
            cv2.LINE_AA
        )
        cv2.putText(
            canvas,
            "Black = obstacle | Red area = inflated safety buffer | Blue = path",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (50, 50, 50),
            1,
            cv2.LINE_AA
        )

        cv2.imshow(self.window_name, canvas)

    def close(self):
        cv2.destroyAllWindows()
