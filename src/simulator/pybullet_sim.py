# pylint: disable=no-member, unused-argument, missing-module-docstring
# pylint: disable=missing-class-docstring, missing-function-docstring

import math
import time
import pybullet as p
import pybullet_data


class PyBulletSimulator:
    def __init__(self, gui=True):
        self.gui = gui
        self.client = None
        self.husky = None

        self.left_wheels = [2, 4]
        self.right_wheels = [3, 5]

        self.wheel_base = 0.55
        self.wheel_radius = 0.165

        self.obstacles = []

    def connect(self):
        if self.gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)

        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

    def load_world(self):
        p.loadURDF("plane.urdf")
        self.husky = p.loadURDF("husky/husky.urdf", [0, 0, 0.1])

        for _ in range(240):
            p.stepSimulation()
            time.sleep(1 / 240)

    def add_box_obstacle(self, position, half_extents):
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=half_extents
        )

        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
            rgbaColor=[1, 0, 0, 1]
        )

        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )

        self.obstacles.append({
            "id": obstacle_id,
            "position": position,
            "half_extents": half_extents
        })

    def create_obstacle_course(self):
        self.add_box_obstacle(position=[-2.0, 0.0, 0.5], half_extents=[0.3, 1.0, 0.5])   # left
        self.add_box_obstacle(position=[2.0, 0.0, 0.5], half_extents=[0.3, 1.0, 0.5])    # right
        self.add_box_obstacle(position=[0.0, 2.0, 0.5], half_extents=[0.8, 0.3, 0.5])    # top
        self.add_box_obstacle(position=[0.0, -2.0, 0.5], half_extents=[0.8, 0.3, 0.5])   # bottom

    def get_robot_pose(self):
        pos, orn = p.getBasePositionAndOrientation(self.husky)
        yaw = p.getEulerFromQuaternion(orn)[2]
        return pos[0], pos[1], yaw

    def set_wheel_speeds(self, left_speed, right_speed):
        for wheel in self.left_wheels:
            p.setJointMotorControl2(
                bodyUniqueId=self.husky,
                jointIndex=wheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=left_speed,
                force=100
            )

        for wheel in self.right_wheels:
            p.setJointMotorControl2(
                bodyUniqueId=self.husky,
                jointIndex=wheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=right_speed,
                force=100
            )

    def set_velocity(self, linear, angular):
        left_linear = linear - (angular * self.wheel_base / 2.0)
        right_linear = linear + (angular * self.wheel_base / 2.0)

        left_angular = left_linear / self.wheel_radius
        right_angular = right_linear / self.wheel_radius

        self.set_wheel_speeds(left_angular, right_angular)

    def is_robot_near_obstacle(self, distance_threshold=0.15):
        if self.husky is None:
            return False

        for obs in self.obstacles:
            points = p.getClosestPoints(
                bodyA=self.husky,
                bodyB=obs["id"],
                distance=distance_threshold
            )
            if len(points) > 0:
                return True

        return False

    def is_robot_in_collision(self, distance_threshold=0.03):
        """
        Returns True if the robot is essentially in contact / extremely close to any obstacle.
        """
        if self.husky is None:
            return False

        for obs in self.obstacles:
            points = p.getClosestPoints(
                bodyA=self.husky,
                bodyB=obs["id"],
                distance=distance_threshold
            )
            if len(points) > 0:
                return True

        return False

    def get_front_point(self, distance=0.5):
        """
        Returns a point in front of the robot in world coordinates.
        """
        x, y, yaw = self.get_robot_pose()
        fx = x + distance * math.cos(yaw)
        fy = y + distance * math.sin(yaw)
        return fx, fy

    def stop_robot(self):
        self.set_wheel_speeds(0, 0)

    def step(self):
        p.stepSimulation()
        time.sleep(1 / 240)

    def disconnect(self):
        p.disconnect()
