import rclpy
from rclpy.node import Node

from robo2023_interfaces.srv import Servores
import serial


class ServoNode(Node):

    def __init__(self):
        super().__init__('ServoNode')
        #Create service to start servo from main node
        self.srv = self.create_service(Servores, 'get_servo_res', self.service_callbacK)

        #Set value for esp32 connection
        self.ser = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1) # TO CHANGE



    def service_callbacK(self, req, res):
        print("I receive request")

        #Send servo data to esp32
        self.ser.write(req.servo)

        #loop until servo is finish
        self.ser.write("finish?") # TO CHANGE
        while self.ser.read(10) != "true":
            self.ser.write("finish?")

        res.result = True    
        return res


def main(args=None):
    rclpy.init(args=args)

    servonode = ServoNode()

    rclpy.spin(servonode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()