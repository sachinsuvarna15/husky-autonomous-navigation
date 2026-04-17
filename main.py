from src.simulator.pybullet_sim import PyBulletSimulator
from src.planner.grid_planner import OccupancyGridMap, AStarPlanner
from src.gui.map_gui import MapGUI
from src.controller.motion_controller import MotionController
import cv2
import pybullet as p
import math


def main():
    sim = PyBulletSimulator(gui=True)
    sim.connect()
    sim.load_world()
    sim.create_obstacle_course()

    grid_map = OccupancyGridMap(map_size=14.0, resolution=0.1)
    grid_map.build_from_obstacles(sim.obstacles, inflation_radius=0.35)

    planner = AStarPlanner(grid_map)
    gui = MapGUI(grid_map, planner)
    controller = MotionController()

    last_path_length = 0
    replan_cooldown = 0
    recovery_steps = 0
    recovery_turn_dir = 1.0

    print("Instructions:")
    print("1st click = Start")
    print("2nd click = Goal")
    print("3rd click = Reset and choose new Start")
    print("Press ESC to quit")

    while True:
        x, y, yaw = sim.get_robot_pose()
        gui.update_robot_pose(x, y)

        if replan_cooldown > 0:
            replan_cooldown -= 1

        # Load new path from GUI
        if len(gui.path_world) > 0 and len(gui.path_world) != last_path_length:
            last_path_length = len(gui.path_world)

            if gui.start is not None:
                start_x, start_y = grid_map.grid_to_world(gui.start[0], gui.start[1])

                start_yaw = 0.0
                if len(gui.path_world) > 1:
                    next_x, next_y = gui.path_world[min(1, len(gui.path_world) - 1)]
                    start_yaw = math.atan2(next_y - start_y, next_x - start_x)

                p.resetBasePositionAndOrientation(
                    sim.husky,
                    [start_x, start_y, 0.1],
                    p.getQuaternionFromEuler([0, 0, start_yaw])
                )
                p.resetBaseVelocity(sim.husky, [0, 0, 0], [0, 0, 0])

                for _ in range(30):
                    sim.step()

                controller.set_path(gui.path_world, current_pose=(start_x, start_y))
                print("Controller path loaded.")
            else:
                controller.set_path(gui.path_world)

        if controller.has_path():
            x, y, yaw = sim.get_robot_pose()
            linear, angular, done, low_progress = controller.compute_control(x, y, yaw)

            near_obstacle = sim.is_robot_near_obstacle(distance_threshold=0.12)
            robot_in_contact = sim.is_robot_in_collision(distance_threshold=0.03)

            actual_stuck = low_progress and near_obstacle
            actual_blocked = robot_in_contact

            if recovery_steps > 0:
                sim.set_velocity(-0.12, 0.40 * recovery_turn_dir)
                recovery_steps -= 1

            else:
                if (actual_stuck or actual_blocked) and replan_cooldown == 0:
                    print("Actual obstacle/stuck condition detected. Recovery motion triggered.")

                    if gui.goal is not None:
                        goal_x, goal_y = grid_map.grid_to_world(gui.goal[0], gui.goal[1])
                        goal_heading = math.atan2(goal_y - y, goal_x - x)
                        heading_error = goal_heading - yaw
                        recovery_turn_dir = 1.0 if math.sin(heading_error) >= 0 else -1.0
                    else:
                        recovery_turn_dir = 1.0

                    recovery_steps = 30
                    sim.set_velocity(-0.12, 0.40 * recovery_turn_dir)

                    print("Replanning from current position...")

                    back_x = x - 0.30 * math.cos(yaw)
                    back_y = y - 0.30 * math.sin(yaw)
                    current_gx, current_gy = grid_map.world_to_grid(back_x, back_y)

                    if not grid_map.is_free(current_gx, current_gy):
                        print("Adjusting start position to nearest free cell...")

                        found = False
                        for r in range(1, 6):
                            for dx in range(-r, r + 1):
                                for dy in range(-r, r + 1):
                                    nx = current_gx + dx
                                    ny = current_gy + dy
                                    if grid_map.is_free(nx, ny):
                                        current_gx, current_gy = nx, ny
                                        found = True
                                        break
                                if found:
                                    break
                            if found:
                                break

                    if gui.goal is not None and grid_map.in_bounds(current_gx, current_gy):
                        new_path = planner.plan((current_gx, current_gy), gui.goal)

                        if len(new_path) > 0:
                            gui.path = new_path
                            gui.path_world = planner.path_to_world(new_path)
                            controller.set_path(gui.path_world, current_pose=(x, y))
                            last_path_length = len(gui.path_world)
                            print("New path found.")
                        else:
                            print("No new path found.")

                    controller.low_progress_counter = 0
                    replan_cooldown = 240

                else:
                    sim.set_velocity(linear, angular)

            if done:
                sim.stop_robot()
                controller.clear_path()
                gui.path = []
                gui.path_world = []
                print("Goal reached.")

        gui.show()
        sim.step()

        key = cv2.waitKey(1) & 0xFF

        if key == ord('w'):
            gui.offset_y -= 20
        elif key == ord('s'):
            gui.offset_y += 20
        elif key == ord('a'):
            gui.offset_x -= 20
        elif key == ord('d'):
            gui.offset_x += 20
        elif key == ord('=') or key == ord('+'):
            gui.zoom *= 1.2
        elif key == ord('-'):
            gui.zoom *= 0.8
        elif key == ord('0'):
            gui.zoom = 1.0
            gui.offset_x = 0
            gui.offset_y = 0
            gui.auto_center = True
        elif key == 27:
            break

    gui.close()
    sim.disconnect()


if __name__ == "__main__":
    main()
