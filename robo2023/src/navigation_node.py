import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from robo2023_interfaces.srv import Navres
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult

class NavNode(Node):

    def __init__(self):
        super().__init__('NavNode')
        #Create service to start navigation from main node
        self.srv = self.create_service(Navres, 'get_nav2_res', self.service_callbacK)
        self.navigator = BasicNavigator()

        #Set initial pose of robot
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0
        initial_pose.pose.position.y = 0
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()


    def service_callbacK(self, req, res):
        print("I receive request")

        #Set goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = req.pose.position.x
        goal_pose.pose.position.y = req.pose.position.y
        goal_pose.pose.position.z = req.pose.position.z
        goal_pose.pose.orientation.x = req.pose.pose.orientation.x
        goal_pose.pose.orientation.y = req.pose.pose.orientation.y
        goal_pose.pose.orientation.z = req.pose.pose.orientation.z
        goal_pose.pose.orientation.w = req.pose.pose.orientation.w

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        self.navigator.goToPose(goal_pose)

        i = 0
        while not self.navigator.isTaskComplete():

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose.pose.position.x = -3.0
                    self.navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            res.result = True
            return res
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