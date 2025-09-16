#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from rclpy.executors import ExternalShutdownException
import math
import time

data_print = ("Coxa ", "Femur ", "Tibia ")

class MyNode (Node):
    def __init__(self):
        super().__init__('hectarus_torque_data')
        self.subscriber_torque_coxa1 = self.create_subscription(Wrench, "/coxa1_torque", self.coxa1_torque, 1)
        self.subscriber_torque_coxa2 = self.create_subscription(Wrench, "/coxa2_torque", self.coxa2_torque, 1)
        self.subscriber_torque_coxa3 = self.create_subscription(Wrench, "/coxa3_torque", self.coxa3_torque, 1)
        self.subscriber_torque_coxa4 = self.create_subscription(Wrench, "/coxa4_torque", self.coxa4_torque, 1)
        self.subscriber_torque_coxa5 = self.create_subscription(Wrench, "/coxa5_torque", self.coxa5_torque, 1)
        self.subscriber_torque_coxa6 = self.create_subscription(Wrench, "/coxa6_torque", self.coxa6_torque, 1)

        self.subscriber_torque_femur1 = self.create_subscription(Wrench, "/femur1_torque", self.femur1_torque, 1)
        self.subscriber_torque_femur2 = self.create_subscription(Wrench, "/femur2_torque", self.femur2_torque, 1)
        self.subscriber_torque_femur3 = self.create_subscription(Wrench, "/femur3_torque", self.femur3_torque, 1)
        self.subscriber_torque_femur4 = self.create_subscription(Wrench, "/femur4_torque", self.femur4_torque, 1)
        self.subscriber_torque_femur5 = self.create_subscription(Wrench, "/femur5_torque", self.femur5_torque, 1)
        self.subscriber_torque_femur6 = self.create_subscription(Wrench, "/femur6_torque", self.femur6_torque, 1)

        self.subscriber_torque_tibia1 = self.create_subscription(Wrench, "/tibia1_torque", self.tibia1_torque, 1)
        self.subscriber_torque_tibia2 = self.create_subscription(Wrench, "/tibia1_torque", self.tibia2_torque, 1)
        self.subscriber_torque_tibia3 = self.create_subscription(Wrench, "/tibia1_torque", self.tibia3_torque, 1)
        self.subscriber_torque_tibia4 = self.create_subscription(Wrench, "/tibia1_torque", self.tibia4_torque, 1)
        self.subscriber_torque_tibia5 = self.create_subscription(Wrench, "/tibia1_torque", self.tibia5_torque, 1)
        self.subscriber_torque_tibia6 = self.create_subscription(Wrench, "/tibia1_torque", self.tibia6_torque, 1)

        self.timer = self.create_timer(0.2, self.print_data)

        self.torque_data = [0, 0, 0, 0, 0, 0,\
                            0, 0, 0, 0, 0, 0,\
                            0, 0, 0, 0, 0, 0] #coxa, femur, tibia

    def coxa1_torque(self, message:Wrench):
        self.torque_data[0] = abs(message.torque.z)
    def coxa2_torque(self, message:Wrench):
        self.torque_data[1] = abs(message.torque.z)
    def coxa3_torque(self, message:Wrench):
        self.torque_data[2] = abs(message.torque.z)
    def coxa4_torque(self, message:Wrench):
        self.torque_data[3] = abs(message.torque.z)
    def coxa5_torque(self, message:Wrench):
        self.torque_data[4] = abs(message.torque.z)
    def coxa6_torque(self, message:Wrench):
        self.torque_data[5] = abs(message.torque.z)

    def femur1_torque(self, message:Wrench):
        self.torque_data[6] = abs(message.torque.z)
    def femur2_torque(self, message:Wrench):
        self.torque_data[7] = abs(message.torque.z)
    def femur3_torque(self, message:Wrench):
        self.torque_data[8] = abs(message.torque.z)
    def femur4_torque(self, message:Wrench):
        self.torque_data[9] = abs(message.torque.z)
    def femur5_torque(self, message:Wrench):
        self.torque_data[10] = abs(message.torque.z)
    def femur6_torque(self, message:Wrench):
        self.torque_data[11] = abs(message.torque.z)

    def tibia1_torque(self, message:Wrench):
        self.torque_data[12] = abs(message.torque.z)
    def tibia2_torque(self, message:Wrench):
        self.torque_data[13] = abs(message.torque.z)
    def tibia3_torque(self, message:Wrench):
        self.torque_data[14] = abs(message.torque.z)
    def tibia4_torque(self, message:Wrench):
        self.torque_data[15] = abs(message.torque.z)
    def tibia5_torque(self, message:Wrench):
        self.torque_data[16] = abs(message.torque.z)
    def tibia6_torque(self, message:Wrench):
        self.torque_data[17] = abs(message.torque.z)

    def print_data(self):
        table_str = ("\n----------------------------------------------------------\n"\
        "--------------Maximum Torque Table Data (Nm)--------------\n" \
        "----------------------------------------------------------\n" \
        "|" + f"{'Joint':^20}" + "|" + f"{'Value':^35}" + "|\n"\
        "----------------------------------------------------------\n")

        for i in range(0, 3):
            for j in range(0, 6):
                table_str += "|" + f"{(data_print[i] + str(j+1)):^20}" + "|" + f"{(self.torque_data[j+(i*6)]):<35}" + "|\n"
        self.get_logger().info(table_str)
def main(args=None):
    try:
        rclpy.init(args=args)
        node = MyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()