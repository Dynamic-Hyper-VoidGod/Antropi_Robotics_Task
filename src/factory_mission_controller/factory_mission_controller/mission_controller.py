import math
import random

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import Buffer, TransformListener


class MissionController(Node):
    def __init__(self):
        super().__init__('factory_mission_controller')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map = None
        self.waypoints = []
        self.goal_index = 0
        self.mission_started = False

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(1.0, self.check_ready)

        self.get_logger().info('TF-based random waypoint mission controller started.')

    def map_callback(self, msg):
        self.map = msg

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            return (
                t.transform.translation.x,
                t.transform.translation.y
            )
        except Exception:
            return None

    def generate_waypoints(self, robot):
        m = self.map.info
        data = self.map.data
        clearance = 6
        candidates = []

        for y in range(clearance, m.height - clearance):
            for x in range(clearance, m.width - clearance):
                index = y * m.width + x

                if data[index] != 0:
                    continue

                safe = True
                for dy in range(-clearance, clearance + 1):
                    for dx in range(-clearance, clearance + 1):
                        if data[(y + dy) * m.width + x + dx] != 0:
                            safe = False
                            break
                    if not safe:
                        break

                if safe:
                    wx = m.origin.position.x + (x + 0.5) * m.resolution
                    wy = m.origin.position.y + (y + 0.5) * m.resolution

                    if math.hypot(wx - robot[0], wy - robot[1]) >= 0.8:
                        candidates.append((wx, wy))

        random.seed()
        random.shuffle(candidates)

        selected = []
        for point in candidates:
            if all(math.hypot(point[0] - p[0], point[1] - p[1]) >= 0.9
                   for p in selected):
                selected.append(point)
            if len(selected) == 3:
                break

        return selected

    def check_ready(self):
        if self.mission_started:
            return

        robot = self.get_robot_pose()
        if self.map is None or robot is None:
            self.get_logger().info('Waiting for map and TF localisation...')
            return

        self.waypoints = self.generate_waypoints(robot)

        if not self.waypoints:
            self.get_logger().error('No safe waypoints found. Check map coverage.')
            return

        self.mission_started = True
        self.timer.cancel()

        self.get_logger().info(
            f'Generated {len(self.waypoints)} safe waypoints: {self.waypoints}'
        )
        self.send_next_goal()

    def send_next_goal(self):
        if self.goal_index >= len(self.waypoints):
            self.get_logger().info('MISSION COMPLETE: all waypoints reached.')
            return

        robot = self.get_robot_pose()
        if robot is None:
            self.get_logger().error('TF unavailable; cannot calculate goal heading.')
            return

        x, y = self.waypoints[self.goal_index]
        yaw = math.atan2(y - robot[1], x - robot[0])

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )

        self.get_logger().info(
            f'Sending waypoint {self.goal_index + 1}/{len(self.waypoints)}: '
            f'x={x:.2f}, y={y:.2f}'
        )

        if not self.navigator.wait_for_server(timeout_sec=0.5):
            self.get_logger().info('Waiting for Nav2 action server...')
            self.create_timer(1.0, self.retry_goal_server)
            return

        future = self.navigator.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def retry_goal_server(self):
        if self.navigator.server_is_ready():
            self.send_next_goal()
            return
        self.get_logger().info('Still waiting for Nav2...')

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Waypoint rejected by Nav2.')
            return

        self.get_logger().info('Waypoint accepted; navigating.')
        result = handle.get_result_async()
        result.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback):
        distance = feedback.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f} m')

    def result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info(
                f'Waypoint {self.goal_index + 1} reached successfully.'
            )
            self.goal_index += 1
            self.send_next_goal()
        else:
            self.get_logger().error(
                f'Waypoint failed. Nav2 status: {status}.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()