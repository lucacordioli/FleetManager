"""!@package rosDialog
@file rosDialog/publisher.py
@brief Contains functions related to the comunication with ROS.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lm_interfaces.msg import Request


class Publisher(Node):
    """!@brief Publishes the request to ROS."""

    def __init__(self):
        """!@brief Initialize the publisher with node telegram_bot and topic telegram_request."""
        super().__init__('telegram_bot')
        self.publisher_ = self.create_publisher(Request, 'telegram_request', 10)

    def publishRequest(self, request):
        """!@brief Parse data and publish the request to ROS.
        @param request: Request to publish.
        """
        msg = Request()
        msg.pickup_name = request['pickup']['name']
        msg.pickup_id = int(request['pickup']['id'])
        msg.dest_name = request['destination']['name']
        msg.dest_id = int(request['destination']['id'])
        msg.slots = int(request['slots'])
        msg.priority = int(request['priority'])

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing')


def initRos():
    """!@brief Initialize the ROS node."""
    rclpy.init(args=None)
    global publisher
    publisher = Publisher()



def destroyRos():
    """!@brief Destroy the ROS node."""
    publisher.destroy_node()
    rclpy.shutdown()
