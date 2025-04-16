#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import math
import statistics
import numpy as np

class TurtlebotExplorerNode(Node):
    def __init__(self):
        super().__init__("explorer_node")
        self.get_logger().info("Explorer Node has started.")
        
        # Declare parameters (for tuning your robot behavior)
        self.declare_parameter("angle_range", 7)
        self.declare_parameter("front_threshold", 1.0)
        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 0.5)
        
        self.angle_range = self.get_parameter("angle_range").value
        self.front_threshold = self.get_parameter("front_threshold").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value

        # Publishers & Subscribers
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._scan_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self._map_sub  = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        
        # Store sensor data for processing
        self.latest_scan = None
        self.latest_map = None

        # Timer: periodically decide what to do (global exploration vs. reactive control)
        self._timer = self.create_timer(1.0, self.exploration_cycle)
        self.current_mode = "REACTIVE"  # can be "REACTIVE" or "GLOBAL"
        
    def laser_callback(self, scan: LaserScan):
        self.latest_scan = scan

    def map_callback(self, map_msg: OccupancyGrid):
        self.latest_map = map_msg

    def process_scan(self, scan: LaserScan):
        """
        Process the LaserScan data:
         - Replace infinite readings with a high (but finite) value.
         - Compute slices for the front, left, and right areas.
         - Return the minimum and median (for future smoothing if needed).
        """
        processed = [r if not math.isinf(r) else 10.0 for r in scan.ranges]
        a = self.angle_range
        front_slice = processed[:a+1] + processed[-a:]
        left_slice  = processed[90-a:90+a+1]
        right_slice = processed[270-a:270+a+1]
        distances = {
            "front": min(front_slice) if front_slice else 10.0,
            "left":  min(left_slice) if left_slice else 10.0,
            "right": min(right_slice) if right_slice else 10.0,
            "front_median": statistics.median(front_slice) if front_slice else 10.0
        }
        self.get_logger().debug(f"Processed distances: {distances}")
        return distances

    def reactive_controller(self, scan: LaserScan):
        """
        A basic reactive controller that uses processed laser data to decide:
         - Move forward if the front is clear.
         - Turn left/right if an obstacle is detected in front.
         - Enter a recovery behavior if the obstacle is very near.
        """
        distances = self.process_scan(scan)
        state = "MOVE_FORWARD"
        if distances["front"] < self.front_threshold:
            # If very close, enter recovery mode.
            if distances["front"] < 0.3:
                state = "RECOVER"
            # Otherwise, decide turn direction based on available space.
            else:
                state = "TURN_LEFT" if distances["left"] > distances["right"] else "TURN_RIGHT"
        return state

    def execute_reactive(self, state: str):
        """Execute a movement command based on the reactive state."""
        cmd = Twist()
        if state == "MOVE_FORWARD":
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        elif state == "TURN_LEFT":
            cmd.linear.x = 0.05
            cmd.angular.z = self.angular_speed
        elif state == "TURN_RIGHT":
            cmd.linear.x = 0.05
            cmd.angular.z = -self.angular_speed
        elif state == "RECOVER":
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
        self._cmd_pub.publish(cmd)
        self.get_logger().info(f"Reactive: {state}")

    def detect_frontier(self, occupancy_grid: OccupancyGrid):
        """
        A very basic frontier detection that finds free cells (value 0) adjacent to unknown cells (value -1).
        For each free cell in the grid, if one of its 8 neighbors is unknown, it's flagged as a frontier.
        Returns the coordinate (row, col) of the first frontier found.
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = np.array(occupancy_grid.data).reshape((height, width))
        
        frontier_cells = []
        # Avoid the borders of the grid
        for i in range(1, height - 1):
            for j in range(1, width - 1):
                if data[i][j] == 0:
                    # Check 8-connected neighborhood
                    neighbors = data[i-1:i+2, j-1:j+2].flatten()
                    if -1 in neighbors:
                        frontier_cells.append((i, j))
        if frontier_cells:
            self.get_logger().info(f"Detected {len(frontier_cells)} frontier cells.")
            # In a complete implementation, you'd choose the best frontier here.
            return frontier_cells[0]
        return None

    def exploration_cycle(self):
        """
        Timer callback that decides between using reactive control or switching to global exploration.
         - If an occupancy grid is available, try to detect a frontier.
         - If a frontier is found, switch to global mode (this is where you would normally integrate a planner).
         - Otherwise, fall back on reactive control using the latest LaserScan.
        """
        # First, try to use global exploration if map data is available.
        if self.latest_map is not None:
            frontier = self.detect_frontier(self.latest_map)
            if frontier is not None:
                # Normally, send a navigation goal computed from frontier data here.
                self.get_logger().info("Frontier detected: switching to global exploration mode.")
                # For demonstration, we simulate global exploration with a simple forward motion.
                cmd = Twist()
                cmd.linear.x = self.linear_speed * 0.5
                cmd.angular.z = 0.0
                self._cmd_pub.publish(cmd)
                self.current_mode = "GLOBAL"
                return  # Exit the cycle; global mode command sent.
        
        # Fallback: if no frontier found or no map available, use reactive obstacle avoidance.
        if self.latest_scan is not None:
            state = self.reactive_controller(self.latest_scan)
            self.execute_reactive(state)
            self.current_mode = "REACTIVE"

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotExplorerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
