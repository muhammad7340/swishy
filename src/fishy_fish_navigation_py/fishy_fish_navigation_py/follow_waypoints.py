import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def define_waypoints(self):
        waypoints = []

        # Keep goals inside the aquarium walls (approximately +/- 2 m).
        wp1 = PoseStamped()
        wp1.header.frame_id = 'map'
        wp1.pose.position.x = 0.8
        wp1.pose.position.y = -0.8
        q = quaternion_from_euler(0, 0, 0)
        wp1.pose.orientation.x = q[0]
        wp1.pose.orientation.y = q[1]
        wp1.pose.orientation.z = q[2]
        wp1.pose.orientation.w = q[3]
        waypoints.append(wp1)

        wp2 = PoseStamped()
        wp2.header.frame_id = 'map'
        wp2.pose.position.x = -0.8
        wp2.pose.position.y = -0.8
        q = quaternion_from_euler(0, 0, 1.57)
        wp2.pose.orientation.x = q[0]
        wp2.pose.orientation.y = q[1]
        wp2.pose.orientation.z = q[2]
        wp2.pose.orientation.w = q[3]
        waypoints.append(wp2)

        wp3 = PoseStamped()
        wp3.header.frame_id = 'map'
        wp3.pose.position.x = -0.8
        wp3.pose.position.y = 0.2
        q = quaternion_from_euler(0, 0, 0)
        wp3.pose.orientation.x = q[0]
        wp3.pose.orientation.y = q[1]
        wp3.pose.orientation.z = q[2]
        wp3.pose.orientation.w = q[3]
        waypoints.append(wp3)

        # Add more waypoints as needed

        return waypoints

    def send_goal(self):
        waypoints = self.define_waypoints()
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info('Waiting for the Nav2 follow_waypoints action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Nav2 follow_waypoints action server is unavailable. Check that '
                'navigation_with_slam.launch.py is running and its lifecycle '
                'manager activated all Nav2 nodes.'
            )
            return False

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_waypoint = feedback.current_waypoint
        self.get_logger().info(f'Navigating to waypoint {current_waypoint}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Waypoint following completed.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    if waypoint_follower.send_goal():
        rclpy.spin(waypoint_follower)
    else:
        waypoint_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
