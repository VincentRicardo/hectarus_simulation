#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

import math

length = 0.1835

class MyNode (Node):
    def __init__(self):
        super().__init__('hectarus_imu_data')
        self.subscriber_imu_data = self.create_subscription(Imu, "/imu", self.imu_data, 1)
        self.publish_rpy = self.create_publisher(Float32MultiArray, "/rpy", 1)
        
    def imu_data(self, message:Imu):
        data = Float32MultiArray()
        data.data = [0, 0, 0]
        r = R.from_quat([message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w]) 
        data.data[0], data.data[1], data.data[2] = r.as_euler('xyz', degrees = True)
        #self.get_logger().info(f"Roll: {round(data.data[0], 3)} degree, Pitch: {round(data.data[1],3)} degree, Yaw: {round(data.data[2],3)} degree")
        self.publish_rpy.publish(data)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()