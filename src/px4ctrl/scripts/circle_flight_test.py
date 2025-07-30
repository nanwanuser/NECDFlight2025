#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class CircleFlightTest:
    def __init__(self):
        rospy.init_node('circle_flight_test', anonymous=True)

        # Publisher for position commands
        self.cmd_pub = rospy.Publisher('/position_cmd', PoseStamped, queue_size=10)

        # Subscriber for odometry to get current position
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)

        # Circle parameters
        self.center_x = 0.0  # Circle center X
        self.center_y = 0.0  # Circle center Y
        self.radius = 2.0    # Circle radius (meters)
        self.height = 1.5    # Flight height (meters)
        self.angular_velocity = 0.2  # rad/s, controls how fast the circle is traced

        # Current drone position
        self.current_pos = None
        self.center_set = False

        # Control parameters
        self.rate = rospy.Rate(20)  # 20 Hz
        self.start_time = None

        rospy.loginfo("Circle Flight Test Node Initialized")
        rospy.loginfo(f"Circle parameters: radius={self.radius}m, height={self.height}m, angular_velocity={self.angular_velocity}rad/s")

    def odom_callback(self, msg):
        """Callback to get current drone position"""
        self.current_pos = msg.pose.pose.position

        # Set circle center to current position on first callback
        if not self.center_set and self.current_pos is not None:
            self.center_x = self.current_pos.x
            self.center_y = self.current_pos.y
            self.center_set = True
            rospy.loginfo(f"Circle center set to: ({self.center_x:.2f}, {self.center_y:.2f})")

    def create_pose_msg(self, x, y, z):
        """Create a PoseStamped message"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Keep orientation fixed (no yaw control as per your controller)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        return pose_msg

    def wait_for_takeoff_position(self):
        """Wait for drone to reach takeoff position"""
        rospy.loginfo("Waiting for drone position data...")
        while not rospy.is_shutdown() and not self.center_set:
            rospy.sleep(0.1)

        # Move to starting position (right side of circle)
        start_x = self.center_x + self.radius
        start_y = self.center_y
        start_z = self.height

        rospy.loginfo(f"Moving to start position: ({start_x:.2f}, {start_y:.2f}, {start_z:.2f})")

        # Publish start position for 5 seconds
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 5.0:
            pose_msg = self.create_pose_msg(start_x, start_y, start_z)
            self.cmd_pub.publish(pose_msg)
            self.rate.sleep()

        rospy.loginfo("Starting circle flight...")
        self.start_time = rospy.Time.now()

    def run_circle_flight(self):
        """Main function to execute circle flight"""
        self.wait_for_takeoff_position()

        while not rospy.is_shutdown():
            # Calculate time since start
            if self.start_time is None:
                self.start_time = rospy.Time.now()

            current_time = rospy.Time.now()
            elapsed_time = (current_time - self.start_time).to_sec()

            # Calculate circle position
            angle = self.angular_velocity * elapsed_time
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y + self.radius * math.sin(angle)
            z = self.height

            # Create and publish pose message
            pose_msg = self.create_pose_msg(x, y, z)
            self.cmd_pub.publish(pose_msg)

            # Log progress every 2 seconds
            if int(elapsed_time) % 2 == 0 and elapsed_time > 0:
                completed_circles = elapsed_time * self.angular_velocity / (2 * math.pi)
                rospy.loginfo_throttle(2, f"Flying circle - Time: {elapsed_time:.1f}s, "
                                          f"Completed circles: {completed_circles:.2f}, "
                                          f"Current pos: ({x:.2f}, {y:.2f}, {z:.2f})")

            self.rate.sleep()

    def emergency_hover(self):
        """Emergency hover at current position"""
        if self.current_pos is not None:
            rospy.logwarn("Emergency hover activated!")
            while not rospy.is_shutdown():
                pose_msg = self.create_pose_msg(
                    self.current_pos.x,
                    self.current_pos.y,
                    self.current_pos.z
                )
                self.cmd_pub.publish(pose_msg)
                self.rate.sleep()

def main():
    try:
        circle_test = CircleFlightTest()

        # Wait a bit for everything to initialize
        rospy.sleep(1.0)

        rospy.loginfo("Starting circle flight test in 3 seconds...")
        rospy.loginfo("Press Ctrl+C to stop and hover")
        rospy.sleep(3.0)

        circle_test.run_circle_flight()

    except rospy.ROSInterruptException:
        rospy.loginfo("Circle flight test interrupted by user")
    except KeyboardInterrupt:
        rospy.loginfo("Circle flight test stopped by user")
        # Try to hover at current position
        try:
            circle_test.emergency_hover()
        except:
            pass
    except Exception as e:
        rospy.logerr(f"Error in circle flight test: {e}")

if __name__ == '__main__':
    main()