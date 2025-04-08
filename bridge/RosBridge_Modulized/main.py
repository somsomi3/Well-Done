#!/usr/bin/env python3
import rclpy
import threading
from .ros_node import RobotBridgeNode
from .flask_server import run_flask_server, set_node_reference

# main.py에 로그 추가
def main(args=None):
    print("Starting robot bridge...")  # 표준 출력에도 로그 남기기
    rclpy.init(args=args)
    node = RobotBridgeNode()
    
    # Share node reference with Flask server
    set_node_reference(node)
    
    print("Starting Flask server thread...")
    # Flask server in a separate thread
    flask_thread = threading.Thread(target=run_flask_server)
    flask_thread.daemon = True
    flask_thread.start()
    
    try:
        print("Spinning ROS node...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received...")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        import traceback
        print(traceback.format_exc())
    finally:
        print("Cleaning up...")
        node.destroy_node()
        rclpy.shutdown()
    print("Robot bridge terminated.")

if __name__ == "__main__":
    main()