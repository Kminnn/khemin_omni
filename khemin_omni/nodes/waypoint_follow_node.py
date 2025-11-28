#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
import threading
import sys

class WaypointCommander(Node):

    def __init__(self):
        super().__init__('waypoint_commander')

        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.goal_handle = None
        self.is_navigating = False
        self.stop_thread = False

        # Start user input thread
        self.input_thread = threading.Thread(target=self.user_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info("Waypoint Commander Ready.")

    # ------------------ WAYPOINT DEFINITIONS -------------------------
    def get_waypoints(self):
        wp = []
        
        w1 = PoseStamped()
        w1.header.frame_id = 'map'
        w1.pose.position.x = 6.4158408
        w1.pose.position.y = 12.5043312
        w1.pose.orientation.z = 0.9897345
        w1.pose.orientation.w = 0.1429178
        wp.append(w1)

        w2 = PoseStamped()
        w2.header.frame_id = 'map'
        w2.pose.position.x = 6.4455070
        w2.pose.position.y = 11.6966477
        w2.pose.orientation.z = -0.7738097
        w2.pose.orientation.w = 0.6334180
        wp.append(w2)

        w3 = PoseStamped()
        w3.header.frame_id = 'map'
        w3.pose.position.x = 4.9656986
        w3.pose.position.y = 6.7620813
        w3.pose.orientation.z= 0.9998369
        w3.pose.orientation.w = 0.0180557
        wp.append(w3)

        w4 = PoseStamped()
        w4.header.frame_id = 'map'
        w4.pose.position.x = 5.8246207
        w4.pose.position.y = 12.7131858
        w4.pose.orientation.z= 0.9927630
        w4.pose.orientation.w = 0.1200899
        wp.append(w4)

        return wp

    # ------------------ SEND SINGLE WAYPOINT -------------------------
    def navigate_to(self, index):
        waypoints = self.get_waypoints()

        if index < 1 or index > len(waypoints):
            self.get_logger().error(f"Invalid waypoint index {index}")
            return

        if self.is_navigating:
            self.get_logger().warn("Navigation already in progress! Type 'stop' first.")
            return

        goal = FollowWaypoints.Goal()
        goal.poses = [waypoints[index - 1]]  # SINGLE WAYPOINT ONLY
        goal.number_of_loops = 1             # No looping

        self.get_logger().info(f"Sending robot to waypoint {index}...")

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 FollowWaypoints server not available!")
            return

        self.is_navigating = True

        send_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    # ------------------ ACTION HANDLERS -------------------------
    def feedback_callback(self, msg):
        pass  # You may log feedback if needed

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            self.is_navigating = False
            return

        self.get_logger().info("Goal accepted.")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("Reached waypoint.")
        self.is_navigating = False
        self.goal_handle = None

    # ------------------ STOP NAVIGATION -------------------------
    def stop_navigation(self):
        if not self.is_navigating:
            self.get_logger().info("Robot is not moving.")
            return

        self.get_logger().info("Stopping robot...")

        if self.goal_handle:
            self.goal_handle.cancel_goal_async()

        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_vel_pub.publish(stop)

        self.is_navigating = False
        self.goal_handle = None

    # ------------------ USER INPUT THREAD -------------------------
    def user_input_loop(self):
        self.get_logger().info(
            "\n--- COMMANDS ---\n"
            "go X     → go to waypoint number X (1,2,3,...)\n"
            "stop     → stop robot immediately\n"
            "exit     → quit program\n"
            "-----------------\n"
        )

        while rclpy.ok() and not self.stop_thread:
            cmd = sys.stdin.readline().strip().lower()

            if cmd.startswith("go"):
                try:
                    index = int(cmd.split()[1])
                    self.navigate_to(index)
                except:
                    self.get_logger().error("Usage: go <waypoint_number>")

            elif cmd == "stop":
                self.stop_navigation()

            elif cmd in ("exit", "quit"):
                rclpy.shutdown()
                return

    # ------------------ CLEAN SHUTDOWN -------------------------
    def shutdown(self):
        self.stop_thread = True
        if self.input_thread.is_alive():
            self.input_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointCommander()

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
