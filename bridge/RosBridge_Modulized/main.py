#!/usr/bin/env python3
import rclpy
import threading
from .ros_node import RobotBridgeNode
from .flask_server import run_flask_server, set_node_reference

def main(args=None):
    rclpy.init(args=args)
    node = RobotBridgeNode()
    
    # Share node reference with Flask server
    set_node_reference(node)
    
    # Flask server in a separate thread
    flask_thread = threading.Thread(target=run_flask_server)
    flask_thread.daemon = True
    flask_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()