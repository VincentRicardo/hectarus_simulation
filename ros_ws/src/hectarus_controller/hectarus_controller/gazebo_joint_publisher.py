#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.executors import ExternalShutdownException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time
from array import array
coxa = 2.5
femur = 6.5
tibia = 8

sleep = 0.4

def h_value(strafe, height):
    h = tibia - height
    alpha1 = math.acos(((femur*tibia)-(math.sqrt((math.pow(femur,2)*math.pow(tibia,2))+(math.pow(femur,2)*(math.pow(h,2)-math.pow(tibia,2))))))/math.pow(femur,2))
    l = math.sin(alpha1)*femur
    return h, l

def right_leg_inverse_kinematic(forward, strafe, height, degree, turn): # degree = 45 -> front leg, -45 -> rear leg
    h,l = h_value(strafe, height)
    angle_base_coxa_right = math.degrees(math.atan2((forward+(math.sin(math.radians(degree))*(l+coxa))), (strafe+(math.cos(math.radians(degree))*(l+coxa))))) - degree
    P = (strafe+(math.cos(math.radians(degree))*(l+coxa)))/math.cos(math.radians(angle_base_coxa_right+degree))
    P = P-coxa
    M = math.sqrt(math.pow(h,2)+math.pow(P,2))

    angle_coxa_femur_1_right = math.degrees(math.acos((math.pow(femur,2)+math.pow(M,2)-math.pow(tibia,2))/(2*femur*M)))
    angle_coxa_femur_2_right = math.degrees(math.acos((math.pow(M,2)+math.pow(h,2)-math.pow(P,2))/(2*h*M)))
    angle_coxa_femur_total = 90-(angle_coxa_femur_1_right + angle_coxa_femur_2_right)
    angle_femur_tibia_right = 90-(math.degrees(math.acos((math.pow(femur,2)+math.pow(tibia,2)-math.pow(M,2))/(2*tibia*femur))))
    return (math.radians(angle_base_coxa_right+turn),(math.radians(angle_coxa_femur_total)), math.radians(angle_femur_tibia_right))

def left_leg_inverse_kinematic(forward, strafe, height, degree, turn): # degree = 45 -> front leg, -45 -> rear leg
    h,l = h_value(strafe, height)
    angle_base_coxa_left = (math.degrees(math.atan2((forward+(math.sin(math.radians(degree))*(l+coxa))), (strafe+(math.cos(math.radians(degree))*(l+coxa))))) - degree)
    P = (strafe+(math.cos(math.radians(degree))*(l+coxa)))/math.cos(math.radians((angle_base_coxa_left+degree)))
    P = P-coxa
    M = math.sqrt(math.pow(h,2)+math.pow(P,2))
    angle_coxa_femur_1_left = math.degrees(math.acos((math.pow(femur,2)+math.pow(M,2)-math.pow(tibia,2))/(2*femur*M)))
    angle_coxa_femur_2_left = math.degrees(math.acos((math.pow(M,2)+math.pow(h,2)-math.pow(P,2))/(2*h*M)))
    angle_coxa_femur_total = -(90-(angle_coxa_femur_1_left + angle_coxa_femur_2_left))
    angle_femur_tibia_left = -(90-(math.degrees(math.acos((math.pow(femur,2)+math.pow(tibia,2)-math.pow(M,2))/(2*tibia*femur)))))
    return -1*math.radians(angle_base_coxa_left-turn), math.radians(angle_coxa_femur_total), math.radians(angle_femur_tibia_left)


class MyNode (Node):
    def __init__(self):
        super().__init__('gazebo_joint_publisher')
        self.subscriber_joint = self.create_subscription(Float32MultiArray, "/cmd_movement", self.get_data, 1)
        self.publish_gazebo_joint = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 1)
        self.subscriber_rpy = self.create_subscription(Float32MultiArray, "/rpy", self.idle,1)
        self.declare_parameter('use_imu', False)
        self.h_mid = 0
        self.h1_rear_front = 0
        self.h2_rear_front = 0

    def send_data(self, trajectory_array, point):
        trajectory_array.points = []
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 40000000
        trajectory_array.points.append(point)
        self.publish_gazebo_joint.publish(trajectory_array)

    def idle(self, message=Float32MultiArray):
        use_imu = self.get_parameter('use_imu').get_parameter_value().bool_value
        if use_imu:
            self.h_mid = (math.tan(math.radians(min(max(message.data[0],-30),20)))*18.35)
            self.h1_rear_front = (math.tan(math.radians(min(max(message.data[0],-30),20)))*12.975) #roll
            self.h2_rear_front = (math.tan(math.radians(min(max(message.data[1],-30),20)))*14.476) #yaw
        else:
            self.h_mid = 0
            self.h1_rear_front = 0
            self.h2_rear_front = 0
        trajectory_msgs = JointTrajectory()
        trajectory_msgs.joint_names = ['body_coxa1_joint', 'coxa1_femur1_joint', 'femur1_tibia1_joint', 'body_coxa2_joint', 'coxa2_femur2_joint', 'femur2_tibia2_joint',\
                            'body_coxa3_joint', 'coxa3_femur3_joint', 'femur3_tibia3_joint', 'body_coxa4_joint', 'coxa4_femur4_joint', 'femur4_tibia4_joint',\
                            'body_coxa5_joint', 'coxa5_femur5_joint', 'femur5_tibia5_joint', 'body_coxa6_joint', 'coxa6_femur6_joint', 'femur6_tibia6_joint']
        point = JointTrajectoryPoint()
        point.positions = [0, 0, 0, -0, -0, -0,\
                        0, 0, 0, -0, -0, -0,\
                        0, 0, 0, -0, -0, -0]
        point.positions[0:3] = array('d', left_leg_inverse_kinematic(0, 0, max(min(self.h1_rear_front - self.h2_rear_front, 4.5),-6), 45, 0)) #1
        point.positions[3:6] = array('d', left_leg_inverse_kinematic(0, 0, max(min(self.h_mid, 4.5),-6), 0, 0)) #2
        point.positions[6:9] = array('d', left_leg_inverse_kinematic(0, 0, max(min(self.h1_rear_front + self.h2_rear_front, 4.5),-6), -45, 0)) #3
        point.positions[9:12] = array('d', right_leg_inverse_kinematic(0, 0, max(min(-self.h1_rear_front + self.h2_rear_front, 4.5), -6), -45, 0)) #4
        point.positions[12:15] = array('d', right_leg_inverse_kinematic(0, 0, max(min(-self.h_mid, 4.5), -6), 0, 0)) #5
        point.positions[15:18] = array('d', right_leg_inverse_kinematic(0, 0, max(min(-self.h1_rear_front - self.h2_rear_front, 4.5), -6), 45, 0)) #6
        self.send_data(trajectory_msgs, point)

    def get_data(self, message:Float32MultiArray):
        trajectory_msgs = JointTrajectory()
        trajectory_msgs.joint_names = ['body_coxa1_joint', 'coxa1_femur1_joint', 'femur1_tibia1_joint', 'body_coxa2_joint', 'coxa2_femur2_joint', 'femur2_tibia2_joint',\
                            'body_coxa3_joint', 'coxa3_femur3_joint', 'femur3_tibia3_joint', 'body_coxa4_joint', 'coxa4_femur4_joint', 'femur4_tibia4_joint',\
                            'body_coxa5_joint', 'coxa5_femur5_joint', 'femur5_tibia5_joint', 'body_coxa6_joint', 'coxa6_femur6_joint', 'femur6_tibia6_joint']
        point = JointTrajectoryPoint()
        point.positions = [0, 0, 0, -0, -0, -0,\
                        0, 0, 0, -0, -0, -0,\
                        0, 0, 0, -0, -0, -0]
        if message.data[0] == 0.0 and message.data[1] == 0.0 and message.data[2] == 0.0: #stop
            self.get_logger().info("Stop")
            point.positions[0:3] = array('d', left_leg_inverse_kinematic(0, 0, max(min((self.h1_rear_front - self.h2_rear_front), 4.5),-6), 45, 0)) #1
            point.positions[3:6] = array('d', left_leg_inverse_kinematic(0, 0, max(min(self.h_mid, 4.5),-6), 0, 0)) #2
            point.positions[6:9] = array('d', left_leg_inverse_kinematic(0, 0, max(min((self.h1_rear_front + self.h2_rear_front), 4.5),-6), -45, 0)) #3
            point.positions[9:12] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h1_rear_front + self.h2_rear_front), 4.5), -6), -45, 0)) #4
            point.positions[12:15] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h_mid), 4.5), -6), 0, 0)) #5
            point.positions[15:18] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h1_rear_front - self.h2_rear_front), 4.5), -6), 45, 0)) #6
            self.send_data(trajectory_msgs, point)

        else:
            #--------------------Move Sequence--------------------#
            self.get_logger().info("Forward")
            #-----Lift----#
            point.positions[0:3] = array('d', left_leg_inverse_kinematic(0, 0, max(min((self.h1_rear_front - self.h2_rear_front)+4.5+ (2*message.data[2]/(message.data[2]+0.1)), 4.5),-6), 45, message.data[2])) #1
            point.positions[3:6] = array('d', left_leg_inverse_kinematic(0, 0, max(min(self.h_mid, 4.5),-6), 0, 0)) #2
            point.positions[6:9] = array('d', left_leg_inverse_kinematic(0, 0, max(min((self.h1_rear_front + self.h2_rear_front)+4.5+ (2*message.data[2]/(message.data[2]+0.1)), 4.5),-6), -45, message.data[2])) #3
            point.positions[9:12] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h1_rear_front + self.h2_rear_front), 4.5), -6), -45, 0)) #4
            point.positions[12:15] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h_mid)+4.5+ (2*message.data[2]/(message.data[2]+0.1)), 4.5),-6), 0, message.data[2])) #5
            point.positions[15:18] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h1_rear_front - self.h2_rear_front), 4.5), -6), 45, 0)) #6
            self.send_data(trajectory_msgs, point)

            time.sleep(sleep)

            #-----Forward----#
            point.positions[0:3] = array('d', left_leg_inverse_kinematic(message.data[0], -message.data[1], max(min((self.h1_rear_front - self.h2_rear_front), 4.5),-6), 45, message.data[2])) #1
            point.positions[3:6] = array('d', left_leg_inverse_kinematic(0, 0, max(min(self.h_mid, 4.5),-6), 0, 0)) #2
            point.positions[6:9] = array('d', left_leg_inverse_kinematic(message.data[0], -message.data[1], max(min((self.h1_rear_front + self.h2_rear_front), 4.5),-6), -45, message.data[2])) #3
            point.positions[9:12] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h1_rear_front + self.h2_rear_front), 4.5), -6), -45, 0)) #4
            point.positions[12:15] = array('d', right_leg_inverse_kinematic(message.data[0], message.data[1], max(min((-self.h_mid), 4.5), -6), 0, message.data[2])) #5
            point.positions[15:18] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h1_rear_front - self.h2_rear_front), 4.5), -6), 45, 0)) #6
            self.send_data(trajectory_msgs, point)

            time.sleep(sleep)


            #-----Idle & Lift----#
            point.positions[0:3] = array('d', left_leg_inverse_kinematic(0, 0, max(min((self.h1_rear_front - self.h2_rear_front), 4.5),-6), 45, 0)) #1
            point.positions[3:6] = array('d', left_leg_inverse_kinematic(0, 0, max(min((self.h_mid)+4.5+ (2*message.data[2]/(message.data[2]+0.1)), 4.5),-6), 0, message.data[2])) #2
            point.positions[6:9] = array('d', left_leg_inverse_kinematic(0, 0, max(min((self.h1_rear_front + self.h2_rear_front), 4.5),-6), -45, 0)) #3
            point.positions[9:12] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h1_rear_front + self.h2_rear_front)+4.5+ (2*message.data[2]/(message.data[2]+0.1)), 4.5),-6), -45, message.data[2])) #4
            point.positions[12:15] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h_mid), 4.5), -6), 0, 0)) #5
            point.positions[15:18] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h1_rear_front - self.h2_rear_front)+4.5+ (2*message.data[2]/(message.data[2]+0.1)), 4.5),-6), 45, message.data[2])) #6
            self.send_data(trajectory_msgs, point)


            time.sleep(sleep)

            #-----Forward----#
            point.positions[0:3] = array('d', left_leg_inverse_kinematic(0, 0, max(min((self.h1_rear_front - self.h2_rear_front), 4.5),-6), 45, 0)) #1
            point.positions[3:6] = array('d', left_leg_inverse_kinematic(message.data[0], -message.data[1], max(min(self.h_mid, 4.5),-6), 0, message.data[2])) #2
            point.positions[6:9] = array('d', left_leg_inverse_kinematic(0, 0, max(min((self.h1_rear_front + self.h2_rear_front), 4.5),-6), -45, 0)) #3
            point.positions[9:12] = array('d', right_leg_inverse_kinematic(message.data[0], message.data[1], max(min((-self.h1_rear_front + self.h2_rear_front), 4.5), -6), -45, message.data[2])) #4
            point.positions[12:15] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h_mid), 4.5), -6), 0, 0)) #5
            point.positions[15:18] = array('d', right_leg_inverse_kinematic(message.data[0], message.data[1], max(min((-self.h1_rear_front - self.h2_rear_front), 4.5), -6), 45, message.data[2])) #6
            self.send_data(trajectory_msgs, point)

            time.sleep(sleep)

            #-----Idle----#
            point.positions[0:3] = array('d', left_leg_inverse_kinematic(0, 0, max(min((self.h1_rear_front - self.h2_rear_front), 4.5),-6), 45, 0)) #1
            point.positions[3:6] = array('d', left_leg_inverse_kinematic(0, 0, max(min(self.h_mid, 4.5),-6), 0, 0)) #2
            point.positions[6:9] = array('d', left_leg_inverse_kinematic(0, 0, max(min((self.h1_rear_front + self.h2_rear_front), 4.5),-6), -45, 0)) #3
            point.positions[9:12] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h1_rear_front + self.h2_rear_front), 4.5), -6), -45, 0)) #4
            point.positions[12:15] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h_mid), 4.5), -6), 0, 0)) #5
            point.positions[15:18] = array('d', right_leg_inverse_kinematic(0, 0, max(min((-self.h1_rear_front - self.h2_rear_front), 4.5), -6), 45, 0)) #6
            self.send_data(trajectory_msgs, point)


        
    
def main(args=None):
    try:
        rclpy.init(args=args)
        node = MyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()