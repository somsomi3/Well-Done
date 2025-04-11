from ssafy_msgs.msg import StatusStamped
from rclpy.node import Node
from builtin_interfaces.msg import Time


def make_status_msg(
    node: Node, tag: str, success: bool, stamp: Time = None
) -> StatusStamped:
    msg = StatusStamped()
    msg.stamp = stamp if stamp else node.get_clock().now().to_msg()
    msg.tag = tag
    msg.status = success
    return msg
