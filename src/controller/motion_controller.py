# pylint: disable=no-member, unused-argument, missing-module-docstring
# pylint: disable=missing-class-docstring, missing-function-docstring

import math


class MotionController:
    def __init__(self):
        self.path_world = []
        self.current_index = 0

        # Slightly faster but still stable
        self.k_linear = 2.0
        self.k_angular = 3.0

        self.max_linear = 6.0
        self.max_angular = 6.0

        self.waypoint_tolerance = 0.35
        self.goal_tolerance = 0.40

        # Progress monitoring
        self.prev_x = None
        self.prev_y = None
        self.low_progress_counter = 0
        self.stuck_threshold = 80
        self.stuck_distance = 0.005

        # Grace period after a new path is loaded
        self.grace_counter = 0
        self.grace_period = 80

    def set_path(self, path_world, current_pose=None):
        self.path_world = path_world
        self.current_index = 0

        self.prev_x = None
        self.prev_y = None
        self.low_progress_counter = 0
        self.grace_counter = self.grace_period

        # Skip very close initial waypoints
        if current_pose is not None and len(self.path_world) > 0:
            x, y = current_pose
            while self.current_index < len(self.path_world):
                tx, ty = self.path_world[self.current_index]
                dist = math.sqrt((tx - x) ** 2 + (ty - y) ** 2)

                if dist < self.waypoint_tolerance:
                    self.current_index += 1
                else:
                    break

            if self.current_index >= len(self.path_world):
                self.current_index = max(0, len(self.path_world) - 1)

    def clear_path(self):
        self.path_world = []
        self.current_index = 0

        self.prev_x = None
        self.prev_y = None
        self.low_progress_counter = 0
        self.grace_counter = 0

    def has_path(self):
        return len(self.path_world) > 0 and self.current_index < len(self.path_world)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def compute_control(self, x, y, yaw):
        if not self.has_path():
            return 0.0, 0.0, True, False

        tx, ty = self.path_world[self.current_index]

        dx = tx - x
        dy = ty - y
        distance = math.sqrt(dx * dx + dy * dy)

        # Switch waypoint if close enough
        if distance < self.waypoint_tolerance:
            self.current_index += 1

            if self.current_index >= len(self.path_world):
                return 0.0, 0.0, True, False

            tx, ty = self.path_world[self.current_index]
            dx = tx - x
            dy = ty - y
            distance = math.sqrt(dx * dx + dy * dy)

        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - yaw)

        angular = self.k_angular * heading_error
        angular = max(-self.max_angular, min(self.max_angular, angular))

        linear = min(self.max_linear, self.k_linear * distance)

        # Smoother turning behavior
        if abs(heading_error) > 1.2:
            linear = 0.0
        elif abs(heading_error) > 0.7:
            linear *= 0.35
        elif abs(heading_error) > 0.3:
        	linear *= 0.7

        # Grace period after loading a path
        if self.grace_counter > 0:
            self.grace_counter -= 1
            self.prev_x = x
            self.prev_y = y
            return linear, angular, False, False

        low_progress = False

        # Only monitor progress when robot should actually move
        if linear > 0.08:
            if self.prev_x is not None and self.prev_y is not None:
                move_dist = math.sqrt((x - self.prev_x) ** 2 + (y - self.prev_y) ** 2)

                if move_dist < self.stuck_distance:
                    self.low_progress_counter += 1
                else:
                    self.low_progress_counter = 0

            self.prev_x = x
            self.prev_y = y

            if self.low_progress_counter > self.stuck_threshold:
                low_progress = True
        else:
            self.low_progress_counter = 0
            self.prev_x = x
            self.prev_y = y

        goal_x, goal_y = self.path_world[-1]
        goal_dist = math.sqrt((goal_x - x) ** 2 + (goal_y - y) ** 2)

        if goal_dist < self.goal_tolerance and self.current_index >= len(self.path_world) - 1:
            return 0.0, 0.0, True, False

        return linear, angular, False, low_progress

    def get_recovery_command(self):
        return -0.25, 0.8
