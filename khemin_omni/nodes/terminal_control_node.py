#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
import threading
import sys
import time

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')

        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.is_navigating = False
        self.current_waypoint = 0  # Index to resume from
        self.number_of_loops = 3

        self.goal_handle = None

        # Start user input thread
        self.input_thread = threading.Thread(target=self.get_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        # Start navigation loop
        self.send_goal()

    def create_waypoints(self):
        waypoint1 = PoseStamped()
        waypoint1.header.frame_id = 'map'
        waypoint1.pose.position.x = 0.158
        waypoint1.pose.position.y = 7.303
        waypoint1.pose.orientation.w = 0.7359

        waypoint2 = PoseStamped()
        waypoint2.header.frame_id = 'map'
        waypoint2.pose.position.x = 4.948
        waypoint2.pose.position.y = 7.303
        waypoint2.pose.orientation.w = -0.7198

        waypoint3 = PoseStamped()
        waypoint3.header.frame_id = 'map'
        waypoint3.pose.position.x = 2.855
        waypoint3.pose.position.y = 3.314
        waypoint3.pose.orientation.w = -0.9986

        return [waypoint1, waypoint2, waypoint3]

    def send_goal(self):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.goal_index = self.current_waypoint
        goal_msg.poses = self.create_waypoints()
        goal_msg.number_of_loops = self.number_of_loops

        self.get_logger().info(f'Sending goal starting at waypoint index {self.current_waypoint}...')

        self.is_navigating = True
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            self.is_navigating = False
            self.goal_handle = None
            return

        self.get_logger().info('Goal accepted by server')
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Current waypoint: {feedback.current_waypoint}')
        self.current_waypoint = feedback.current_waypoint

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status: {status}')
        self.is_navigating = False
        self.goal_handle = None

    def cancel_navigation(self):
        if self.is_navigating and self.goal_handle:
            self.get_logger().info('Cancelling current navigation task...')
            cancel_future = self.goal_handle.cancel_goal_async()
            # Publish zero velocity to stop robot immediately
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)
            self.is_navigating = False
            self.goal_handle = None
            self.get_logger().info(f'Navigation cancelled at waypoint index {self.current_waypoint}')        

    def resume_navigation(self):
        self.get_logger().info(f'Resuming navigation from waypoint index {self.current_waypoint}')
        self.send_goal()

    def get_user_input(self):
        self.get_logger().info(
            '\n--- TERMINAL CONTROL READY ---\n'
            'Type "stop" to halt motion.\n'
            'Type "go" to resume navigation.\n'
            'Type "exit" or "quit" to close the node.\n'
            '--------------------------------\n'
        )
        while rclpy.ok():
            try:
                user_input = sys.stdin.readline().strip().lower()
                if user_input == 'stop':
                    self.cancel_navigation()
                elif user_input == 'go':
                    self.resume_navigation()
            except Exception as e:
                self.get_logger().error(f'Error reading input: {e}')
                break

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
