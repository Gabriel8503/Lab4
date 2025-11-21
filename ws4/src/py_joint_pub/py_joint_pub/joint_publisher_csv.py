import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs
import pkg_resources
import numpy as np
import os

class JointPublisherCSV(Node):

    def __init__(self):
        super().__init__('joint_publisher_csv')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.trail_pub = self.create_publisher(Marker, 'led_trail', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        timer_period = 1/500 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        filename = 'gturak_bonus.csv' # change this line of code. The provided CSV file (ldihel.csv) is incomplete
        csv_file = pkg_resources.resource_filename('py_joint_pub', f'../resource/{filename}')
        self.get_logger().info(f"Found CSV file {csv_file}")
        self.csv_data = np.genfromtxt(csv_file, delimiter=',', skip_header=1)
        self.data_length = len(self.csv_data)
        self.get_logger().info(f"Read CSV file {csv_file} \
            with {np.size(self.csv_data,0)} rows and {np.size(self.csv_data,1)} columns")


        self.trail_marker = Marker()
        self.trail_marker.header.frame_id = "world"
        self.trail_marker.ns = "led_trail"
        self.trail_marker.id = 0
        self.trail_marker.type = Marker.POINTS
        self.trail_marker.action = Marker.ADD
        self.trail_marker.scale.x = 0.005
        self.trail_marker.scale.y = 0.005
        self.trail_marker.color.r = 1.0
        self.trail_marker.color.g = 0.0
        self.trail_marker.color.b = 0.0
        self.trail_marker.color.a = 1.0

    def timer_callback(self):
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        msg.position = list(self.csv_data[self.i, 1:7])
        
        msg.velocity = []
        msg.effort = []
        
        self.publisher_.publish(msg)
        
        led_on = int(self.csv_data[self.i, 7]) == 1
        
        if led_on:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "world",       
                    "tool0",
                    rclpy.time.Time()
                )

                p = Point()
                p.x = transform.transform.translation.x
                p.y = transform.transform.translation.y
                p.z = transform.transform.translation.z

                self.trail_marker.points.append(p)
                self.trail_marker.header.stamp = self.get_clock().now().to_msg()
                self.trail_pub.publish(self.trail_marker)

            except Exception as e:
                self.get_logger().warn(f"TF lookup failed: {e}")

        
        self.i += 1
        self.i %= self.data_length # loop back to the beginning of the csv file


def main(args=None):
    rclpy.init(args=args)

    node = JointPublisherCSV()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
