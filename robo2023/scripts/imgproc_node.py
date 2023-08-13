#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from robo2023_interfaces.srv import Imgres
import cv2

class ImgNode(Node):

    def __init__(self):
        super().__init__('ImgNode')
        #Create service to start servo from main node
        self.srv = self.create_service(Imgres, 'get_img_res', self.service_callbacK)


    def service_callbacK(self, req, res):
        print("I receive request")

        #Image Processing ..........

        result = self.imgproc()

        res.result = result   
        return res
    
    def imgproc(self):
        return "string"


def main(args=None):
    rclpy.init(args=args)

    imgnode = ImgNode()

    rclpy.spin(imgnode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()