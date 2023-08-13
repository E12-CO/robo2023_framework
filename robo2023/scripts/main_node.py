#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from robo2023_interfaces.srv import Navres
from robo2023_interfaces.srv import Imgres
from robo2023_interfaces.srv import Servores

from geometry_msgs.msg import Pose

class MainNode(Node):

    def __init__(self):
        super().__init__('MainNode')

        #Nav2 client
        self.nav_cli = self.create_client(Navres, 'get_nav2_res')       # CHANGE
        while not self.nav_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Nav2 service not available, waiting again...')
        self.nav_req = Navres.Request()                                   # CHANGE

        #Imgproc Client
        self.img_cli = self.create_client(Imgres, 'get_img_res')       # CHANGE
        while not self.img_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Img service not available, waiting again...')
        self.img_req = Imgres.Request()                                   # CHANGE

        #Servo Client
        self.servo_cli = self.create_client(Servores, 'get_servo_res')       # CHANGE
        while not self.servo_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servo service not available, waiting again...')
        self.servo_req = Servores.Request()                                   # CHANGE

    #Send Nav2 pose to nav2 node
    def send_nav_request(self,pose):
        self.nav_req.pose = pose                # CHANGE
        self.future = self.nav_cli.call_async(self.nav_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    #Send Imgproc signal to imgproc node
    def send_img_request(self,signal):
        self.img_req.img = signal                # CHANGE
        self.future = self.img_cli.call_async(self.img_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    #Send Servo signal to servo node
    def send_servo_request(self,signal):
        self.servo_req.servo = signal                # CHANGE
        self.future = self.servo_cli.call_async(self.servo_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    mainnode = MainNode()

    #Fix position of object
    pos1 = Pose()
    pos2 = Pose()
    pos3 = Pose()
    pos4 = Pose()
    pos5 = Pose()

    #Go to mission board
    mission_pose = Pose()
    go_to_mission = mainnode.send_nav_request(mission_pose)
    if go_to_mission.result == False:
        mission_pose.get_logger().info("Navigation Failure - Mission Board")
    else:
        mission_pose.get_logger().info("Navigation Succesful - Mission Board")

    #Read the mission
    read_mission = mainnode.send_img_request("read")
    mission_detail = read_mission.result

    #Assign pos1-5 accord to mission
    c_pos1 = pos1
    c_pos2 = pos2
    c_pos3 = pos3
    c_pos4 = pos4
    c_pos5 = pos5

    #Go to point 1
    go_to_point1 = mainnode.send_nav_request(c_pos1)
    if go_to_point1.result == False:
        go_to_point1.get_logger().info("Navigation Failure - Position 1")
    else:
        go_to_point1.get_logger().info("Navigation Succesful - Position 1")

    #Transport object point 1
    transporting_1 = mainnode.send_servo_request("run")
    if transporting_1.result == False:
        transporting_1.get_logger().info("Servo Failure - Position 1")
    else:
        transporting_1.get_logger().info("Servo Succesful - Position 1")

    #Go to point 2
    go_to_point2 = mainnode.send_nav_request(c_pos2)
    if go_to_point2.result == False:
        go_to_point2.get_logger().info("Navigation Failure - Position 2")
    else:
        go_to_point2.get_logger().info("Navigation Succesful - Position 2")

    #Transport object point 2
    transporting_2 = mainnode.send_servo_request("run")
    if transporting_2.result == False:
        transporting_2.get_logger().info("Servo Failure - Position 2")
    else:
        transporting_2.get_logger().info("Servo Succesful - Position 2")

    #Go to point 3
    go_to_point3 = mainnode.send_nav_request(c_pos3)
    if go_to_point3.result == False:
        go_to_point3.get_logger().info("Navigation Failure - Position 3")
    else:
        go_to_point3.get_logger().info("Navigation Succesful - Position 3")

    #Transport object point 3
    transporting_3 = mainnode.send_servo_request("run")
    if transporting_3.result == False:
        transporting_3.get_logger().info("Servo Failure - Position 3")
    else:
        transporting_3.get_logger().info("Servo Succesful - Position 3")

    #Go to point 4
    go_to_point4 = mainnode.send_nav_request(c_pos4)
    if go_to_point4.result == False:
        go_to_point4.get_logger().info("Navigation Failure - Position 4")
    else:
        go_to_point4.get_logger().info("Navigation Succesful - Position 4")

    #Transport object point 4
    transporting_4 = mainnode.send_servo_request("run")
    if transporting_4.result == False:
        transporting_4.get_logger().info("Servo Failure - Position 4")
    else:
        transporting_4.get_logger().info("Servo Succesful - Position 4")

    #Go to point 5
    go_to_point5 = mainnode.send_nav_request(c_pos5)
    if go_to_point5.result == False:
        go_to_point5.get_logger().info("Navigation Failure - Position 5")
    else:
        go_to_point5.get_logger().info("Navigation Succesful - Position 5")

    #Transport object point 5
    transporting_5 = mainnode.send_servo_request("run")
    if transporting_5.result == False:
        transporting_5.get_logger().info("Servo Failure - Position 5")
    else:
        transporting_5.get_logger().info("Servo Succesful - Position 5")


    #Finish , shutdown 
    mainnode.destroy_node()
    rclpy.shutdown()