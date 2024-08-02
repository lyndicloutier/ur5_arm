# pylint: disable=missing-module-docstring, missing-class-docstring, missing-final-newline, missing-function-docstring, trailing-whitespace, line-too-long

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class Publisher(Node):
    def __init__(self):
        super().__init__("publisher_testing")
        self.publisher_ = self.create_publisher(PoseStamped, 'posestamped', 1)
        self.timer_ = self.create_timer(1.0, self.position_callback)

    def position_callback(self):
        msg = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.header.frame_id = "world"
        self.get_logger().info("Publishing: position = (" + str(msg.pose.position.x) + ", " + str(msg.pose.position.y) 
                               + ", " + str(msg.pose.position.z) + ", " + str(msg.pose.orientation.x) 
                               + ", " + str(msg.pose.orientation.y) + ", " + str(msg.pose.orientation.z) 
                               + ") and frame_id = " + msg.header.frame_id)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# publishing once for entire robot or publishing once per joint/tf?
# continuously publishing or just sending the coordinates once?
# figure out how to use tfs and how to store coordinates for each joint in the .yaml file
# ask Jordan about whether the coordinates are being published for each joint/tf
# figure out how the tfs work after testing the publisher and subscriber