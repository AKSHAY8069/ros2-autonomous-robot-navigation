import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class RobotDemo(Node):

    def __init__(self):
        super().__init__('robot_demo')

        # -----------------------------
        # ROS PUBLISHERS
        # -----------------------------

        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map', 10)

        self.path_pub = self.create_publisher(
            Path, '/planned_path', 10)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/robot_markers', 10)

        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', 10)

        # Robot position
        self.robot_x = -4.5
        self.robot_y = -2.5
        self.robot_yaw = 0.0
        self.progress = 0.0

        # Destination
        self.goal_x = 4.5
        self.goal_y = 2.5

        # Timer
        self.timer = self.create_timer(
            0.2, self.publish_demo)

        self.get_logger().info(
            'Robot SLAM demonstration started')

    # ==========================================================
    # MAIN PUBLISH FUNCTION
    # ==========================================================

    def publish_demo(self):

        # Move robot from START to GOAL
        self.progress += 0.002

        if self.progress >= 1.0:
            self.progress = 0.0

        t = self.progress

        # Fixed start and goal
        start_x = -4.5
        start_y = -2.5

        goal_x = 4.5
        goal_y = 2.5

        # Robot position on the planned path
        self.robot_x = start_x + t * (goal_x - start_x)

        self.robot_y = start_y + t * (goal_y - start_y)

        self.robot_y += 0.4 * math.sin(t * math.pi)

        # Calculate robot direction
        next_t = min(t + 0.01, 1.0)

        next_x = start_x + next_t * (goal_x - start_x)
        next_y = start_y + next_t * (goal_y - start_y)

        next_y += 0.4 * math.sin(next_t * math.pi)

        self.robot_yaw = math.atan2(
            next_y - self.robot_y,
            next_x - self.robot_x
        )

        # Publish all visualization data
        self.publish_map()
        self.publish_path()
        self.publish_markers()
        self.publish_lidar()
    # ==========================================================
    # 2-D MAP
    # ==========================================================

    def publish_map(self):

        resolution = 0.1
        width = 120
        height = 80

        grid = OccupancyGrid()

        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'

        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height

        grid.info.origin.position.x = -6.0
        grid.info.origin.position.y = -4.0

        data = [0] * (width * height)

        # -----------------------------
        # OUTER WALLS
        # -----------------------------

        for x in range(width):
            data[x] = 100
            data[(height - 1) * width + x] = 100

        for y in range(height):
            data[y * width] = 100
            data[y * width + width - 1] = 100

        # -----------------------------
        # OBSTACLES
        # -----------------------------

        obstacles = [
            (30, 20, 20, 5),
            (65, 40, 5, 25),
            (80, 15, 25, 5),
            (45, 55, 20, 5)
        ]

        for ox, oy, ow, oh in obstacles:

            for y in range(oy, oy + oh):

                for x in range(ox, ox + ow):

                    if 0 <= x < width and 0 <= y < height:

                        data[y * width + x] = 100

        grid.data = data

        self.map_pub.publish(grid)

    # ==========================================================
    # PLANNED PATH
    # ==========================================================

    def publish_path(self):

        path = Path()

        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        for i in range(101):

            pose = PoseStamped()

            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'

            t = i / 100.0

        # Fixed START position
        start_x = -4.5
        start_y = -2.5

        # Fixed GOAL position
        goal_x = 4.5
        goal_y = 2.5

        # Curved planned path
        x = start_x + t * (goal_x - start_x)

        y = start_y + t * (goal_y - start_y)

        y += 0.4 * math.sin(t * math.pi)

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

        self.path_pub.publish(path)

    # ==========================================================
    # ROBOT + GOAL + WAYPOINTS + ELEVATION
    # ==========================================================

    def publish_markers(self):

        markers = MarkerArray()

        # ------------------------------------------------------
        # ROBOT
        # ------------------------------------------------------

        robot = Marker()

        robot.header.frame_id = 'map'
        robot.header.stamp = self.get_clock().now().to_msg()

        robot.ns = 'robot'
        robot.id = 0

        robot.type = Marker.CYLINDER
        robot.action = Marker.ADD

        robot.pose.position.x = self.robot_x
        robot.pose.position.y = self.robot_y
        robot.pose.position.z = 0.2

        robot.scale.x = 0.5
        robot.scale.y = 0.5
        robot.scale.z = 0.4

        robot.color.r = 0.0
        robot.color.g = 1.0
        robot.color.b = 0.0
        robot.color.a = 1.0

        markers.markers.append(robot)

        # ------------------------------------------------------
        # ROBOT ORIENTATION ARROW
        # ------------------------------------------------------

        arrow = Marker()

        arrow.header.frame_id = 'map'
        arrow.header.stamp = self.get_clock().now().to_msg()

        arrow.ns = 'robot_direction'
        arrow.id = 1

        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        start = Point()
        start.x = self.robot_x
        start.y = self.robot_y
        start.z = 0.3

        end = Point()

        end.x = self.robot_x + 1.0 * math.cos(
            self.robot_yaw)

        end.y = self.robot_y + 1.0 * math.sin(
            self.robot_yaw)

        end.z = 0.3

        arrow.points.append(start)
        arrow.points.append(end)

        arrow.scale.x = 0.12
        arrow.scale.y = 0.2
        arrow.scale.z = 0.2

        arrow.color.r = 0.0
        arrow.color.g = 0.7
        arrow.color.b = 1.0
        arrow.color.a = 1.0

        markers.markers.append(arrow)

        # ------------------------------------------------------
        # GOAL
        # ------------------------------------------------------

        goal = Marker()

        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.ns = 'goal'
        goal.id = 2

        goal.type = Marker.SPHERE
        goal.action = Marker.ADD

        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.position.z = 0.3

        goal.scale.x = 0.6
        goal.scale.y = 0.6
        goal.scale.z = 0.6

        goal.color.r = 1.0
        goal.color.g = 0.0
        goal.color.b = 0.0
        goal.color.a = 1.0

        markers.markers.append(goal)

        # ------------------------------------------------------
        # WAYPOINTS
        # ------------------------------------------------------

        for i in range(1, 10):

            waypoint = Marker()

            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.get_clock().now().to_msg()

            waypoint.ns = 'waypoints'
            waypoint.id = 10 + i

            waypoint.type = Marker.SPHERE
            waypoint.action = Marker.ADD

            t = i / 10.0

            waypoint.pose.position.x = (
                self.robot_x +
                t * (self.goal_x - self.robot_x)
            )

            waypoint.pose.position.y = (
                self.robot_y +
                t * (self.goal_y - self.robot_y)
                + 0.4 * math.sin(t * math.pi)
            )

            waypoint.pose.position.z = 0.12

            waypoint.scale.x = 0.15
            waypoint.scale.y = 0.15
            waypoint.scale.z = 0.15

            waypoint.color.r = 0.0
            waypoint.color.g = 0.5
            waypoint.color.b = 1.0
            waypoint.color.a = 1.0

            markers.markers.append(waypoint)

        # ------------------------------------------------------
        # ELEVATION / 3-D OBSTACLES
        # ------------------------------------------------------

        elevation_objects = [
            (-2.5, -0.5, 0.8, 1.0),
            (0.5, 1.0, 1.2, 1.5),
            (2.5, -1.5, 0.7, 1.2),
            (-1.0, 2.0, 1.0, 0.8)
        ]

        for i, (x, y, z, size) in enumerate(
                elevation_objects):

            building = Marker()

            building.header.frame_id = 'map'
            building.header.stamp = (
                self.get_clock().now().to_msg())

            building.ns = 'elevation'
            building.id = 100 + i

            building.type = Marker.CUBE
            building.action = Marker.ADD

            building.pose.position.x = x
            building.pose.position.y = y
            building.pose.position.z = z / 2.0

            building.scale.x = size
            building.scale.y = size
            building.scale.z = z

            building.color.r = 0.5
            building.color.g = 0.5
            building.color.b = 0.5
            building.color.a = 0.8

            markers.markers.append(building)

        # ------------------------------------------------------
        # DISTANCE TO GOAL TEXT
        # ------------------------------------------------------

        distance = math.sqrt(
            (self.goal_x - self.robot_x) ** 2 +
            (self.goal_y - self.robot_y) ** 2
        )

        text = Marker()

        text.header.frame_id = 'map'
        text.header.stamp = (
            self.get_clock().now().to_msg())

        text.ns = 'information'
        text.id = 200

        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.pose.position.x = 0.0
        text.pose.position.y = 3.4
        text.pose.position.z = 1.5

        text.scale.z = 0.4

        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0

        text.text = (
            'SLAM NAVIGATION | '
            'Distance to Goal: %.2f m' % distance
        )

        markers.markers.append(text)

        self.marker_pub.publish(markers)

    # ==========================================================
    # SIMULATED LiDAR
    # ==========================================================

    def publish_lidar(self):

        scan = LaserScan()

        scan.header.stamp = (
            self.get_clock().now().to_msg())

        scan.header.frame_id = 'map'

        scan.angle_min = -math.pi
        scan.angle_max = math.pi

        scan.angle_increment = math.radians(2)

        scan.range_min = 0.1
        scan.range_max = 6.0

        ranges = []

        number_of_rays = int(
            (scan.angle_max - scan.angle_min) /
            scan.angle_increment
        )

        for i in range(number_of_rays):

            angle = (
                scan.angle_min +
                i * scan.angle_increment
            )

            # Simulated environment
            # creates varying LiDAR distances

            distance = 3.5

            # Front obstacle
            if -0.5 < angle < 0.5:
                distance = 2.0

            # Left obstacle
            elif 1.0 < angle < 1.5:
                distance = 1.5

            # Right obstacle
            elif -1.5 < angle < -1.0:
                distance = 2.2

            ranges.append(distance)

        scan.ranges = ranges

        self.scan_pub.publish(scan)


# ==============================================================
# MAIN
# ==============================================================

def main(args=None):

    rclpy.init(args=args)

    node = RobotDemo()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()