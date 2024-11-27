import os
import csv
import time
from pynput import keyboard
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class WaypointMaker(Node):
    def __init__(self):
        super().__init__('waypoint_maker')

        # Parameters
        self.declare_parameter('saving_waypoints_file_name', '')
        self.saving_waypoints_file_name = self.get_parameter('saving_waypoints_file_name').value

        # Subscription
        self.create_subscription(PoseWithCovarianceStamped, 'current_pose', self.save_pose_callback, 10)

        # Publisher
        self.pose_publisher = self.create_publisher(PoseStamped, '/navigation_manager/waypoint_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint/markers', 10)
        self.text_marker_pub = self.create_publisher(MarkerArray, 'waypoint/text_markers', 10)
        self.line_marker_pub = self.create_publisher(MarkerArray, 'waypoint/line_markers', 10)

        # Variables
        self.poses = []
        self.current_pose = None
        self.pose_id = 0
        self.skip_flag = 1
        self.last_pose = PoseStamped()
        self.last_save_time = time.time()
        self.last_erase_time = time.time()

        # Keyboard save mode
        self.get_logger().info('Press "s" to save the current pose, "q" to quit and save to csv.')
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.start()

        # Marker-related variables
        self.markers = MarkerArray()
        self.text_markers = MarkerArray()
        self.line_markers = MarkerArray()

    def save_pose_callback(self, msg):
        self.get_logger().info('called.')
        self.current_pose = msg.pose.pose
        if self.current_pose is not None:
            pose_data = [
                str(self.pose_id),
                str(self.current_pose.position.x),
                str(self.current_pose.position.y),
                str(self.current_pose.position.z),
                str(self.current_pose.orientation.x),
                str(self.current_pose.orientation.y),
                str(self.current_pose.orientation.z),
                str(self.current_pose.orientation.w),
                int(self.skip_flag)
            ]
            self.poses.append(pose_data)
            self.get_logger().info(f'saved! ID: {self.pose_id}')

            # Publish the saved pose as PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose = self.current_pose
            self.pose_publisher.publish(pose_stamped)

            # Create and publish markers
            self.create_and_publish_markers(pose_stamped)

            # Increment pose ID after publishing the markers
            self.pose_id += 1

            # Memorize the last recorded pose
            self.last_pose.pose = self.current_pose

        else:
            self.get_logger().warn('Warning: No pose received yet.')

    def create_and_publish_markers(self, pose_stamped):
        # Create a marker for the waypoint
        marker = Marker()
        marker.header.frame_id = pose_stamped.header.frame_id
        marker.header.stamp = pose_stamped.header.stamp
        marker.ns = "waypoints"
        marker.id = self.pose_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.markers.markers.append(marker)

        # Create a text marker to display the waypoint ID
        text_marker = Marker()
        text_marker.header.frame_id = pose_stamped.header.frame_id
        text_marker.header.stamp = pose_stamped.header.stamp
        text_marker.ns = "waypoints_text"
        text_marker.id = self.pose_id + 1000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = pose_stamped.pose.position.x + 0.3
        text_marker.pose.position.y = pose_stamped.pose.position.y - 0.3
        text_marker.pose.position.z = pose_stamped.pose.position.z + 0.3
        text_marker.scale.z = 0.5
        text_marker.color.a = 1.0
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.text = str(self.pose_id)
        self.text_markers.markers.append(text_marker)

        # Create a line marker connecting to the previous waypoint, if applicable
        if len(self.markers.markers) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = pose_stamped.header.frame_id
            line_marker.header.stamp = pose_stamped.header.stamp
            line_marker.ns = "waypoints_lines"
            line_marker.id = self.pose_id + 2000
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.02
            line_marker.color.a = 1.0
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0

            # Set the points for the line
            line_marker.points.append(self.markers.markers[-2].pose.position)
            line_marker.points.append(pose_stamped.pose.position)

            self.line_markers.markers.append(line_marker)

        # Publish all markers
        self.marker_pub.publish(self.markers)
        self.text_marker_pub.publish(self.text_markers)
        self.line_marker_pub.publish(self.line_markers)

    def keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_key_press) as listener:
            listener.join()

    def on_key_press(self, key):
        try:
            if key.char == 'q':
                self.save_poses_to_csv()
                self.destroy_node()
                rclpy.shutdown()
        except AttributeError:
            pass

    def save_poses_to_csv(self):
        if self.saving_waypoints_file_name:
            filename = self.saving_waypoints_file_name
        else:
            filename = datetime.now().strftime('%Y%m%d%H%M%S') + '_waypoints.csv'
        
        path = os.path.join(os.path.curdir, 'src', 'waypoint_manager', 'waypoints', filename)  

        os.makedirs(os.path.dirname(path), exist_ok=True)

        with open(path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['id', 'pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w', 'skip_flag'])
            writer.writerows(self.poses)

        self.get_logger().info(f'Saved {len(self.poses)} poses to {path}.')


def main(args=None):
    rclpy.init(args=args)
    pose_recorder = WaypointMaker()

    try:
        rclpy.spin(pose_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()