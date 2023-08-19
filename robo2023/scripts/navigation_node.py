#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from robo2023_interfaces.srv import Navres
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class NavNode(Node):

    def __init__(self):
        super().__init__('NavNode')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        #Create service to start navigation from main node
        self.srv = self.create_service(Navres, 'get_nav2_res', self.service_callbacK)
        #self.nav = self.create_publisher(PoseStamped, 'goal_pose', qos_profile=qos_profile)
        self.nav = BasicNavigator()

        # ...

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        self.nav.setInitialPose(initial_pose)
        self.nav.waitUntilNav2Active(localizer="bt_navigator") # if autostarted, else use lifecycleStartup()


    def service_callbacK(self, req, res):
        print("I receive request")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = req.position.x
        goal_pose.pose.position.y = req.position.y
        goal_pose.pose.position.z = req.position.z
        goal_pose.pose.orientation.x = req.orientation.x
        goal_pose.pose.orientation.y = req.orientation.y
        goal_pose.pose.orientation.z = req.orientation.z
        goal_pose.pose.orientation.w = req.orientation.w
        self.nav.goToPose(goal_pose)

        i = 0
        while not self.nav.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.nav.cancelTask()

        # Do something depending on the return code
        result = self.nav.getResult()
        print(result)
        if result == TaskResult.SUCCEEDED:
            res.result = True
        else:
            res.result = False
        return res
    

def main(args=None):
    rclpy.init(args=args)

    navnode = NavNode()

    rclpy.spin(navnode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()