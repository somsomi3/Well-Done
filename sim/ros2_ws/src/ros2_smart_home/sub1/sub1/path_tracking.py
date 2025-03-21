import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,Point
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt,atan2
import numpy as np

class followTheCarrot(Node):

    """ 
    📌 경로 추종 노드 (Path Tracking)
    - 로봇의 현재 위치(/odom), 속도(/turtlebot_status), 경로(/local_path)를 받아
      전방 주시 포인트를 기준으로 속도 및 방향을 결정하여 /cmd_vel 퍼블리시
    """

    def __init__(self):
        super().__init__('path_tracking')

        # ✅ Publisher & Subscriber 생성
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        self.path_sub = self.create_subscription(Path,'/local_path',self.path_callback,10)

        # ✅ 제어 주기 및 타이머 설정
        time_period=0.05 # 50ms마다 실행
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom=False
        self.is_path=False
        self.is_status=False

        self.odom_msg=Odometry()            
        self.robot_yaw=0.0
        self.path_msg=Path()
        self.cmd_msg=Twist()

        # ✅ 전방 주시 거리 파라미터 설정
        self.lfd=0.1 # Look-Forward Distance
        self.min_lfd=0.1
        self.max_lfd=1.0


    def timer_callback(self):
        """ 경로를 따라가도록 속도를 제어하는 콜백 함수 """
        if self.is_status and self.is_odom and self.is_path:
            if len(self.path_msg.poses) > 1:
                self.is_look_forward_point= False
                
                # ✅ 로봇 현재 위치
                robot_pose_x=self.odom_msg.pose.pose.position.x
                robot_pose_y=self.odom_msg.pose.pose.position.y

                # ✅ 로봇이 경로에서 떨어진 거리 계산
                lateral_error= sqrt(
                    pow(self.path_msg.poses[0].pose.position.x-robot_pose_x,2)+
                    pow(self.path_msg.poses[0].pose.position.y-robot_pose_y,2)
                )
                print(f"현재 위치: x={robot_pose_x}, y={robot_pose_y}, 오차={lateral_error}")
                
                # ✅ 로봇 속도를 기반으로 Look-Forward Distance 설정
                self.lfd = max(self.min_lfd, min(self.max_lfd, lateral_error))

                min_dis=float('inf')

                # ✅ 전방 주시 포인트 찾기
                for num, waypoint in enumerate(self.path_msg.poses):
                    current_point = waypoint.pose.position
                    dis = sqrt(pow(current_point.x - robot_pose_x, 2) + pow(current_point.y - robot_pose_y, 2))

                    if abs(dis - self.lfd) < min_dis:
                        min_dis = abs(dis - self.lfd)
                        self.forward_point = current_point
                        self.is_look_forward_point = True        
                
                if self.is_look_forward_point :
                    global_forward_point=[self.forward_point.x ,self.forward_point.y,1]

                    # ✅ 전방 주시 포인트와 로봇 헤딩 간의 각도 계산
                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                        [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                        [0, 0, 1]
                    ])
                    det_trans_matrix = np.linalg.inv(trans_matrix)
                    local_forward_point = det_trans_matrix.dot(np.array(global_forward_point).reshape(3, 1))

                    theta = atan2(local_forward_point[1][0], local_forward_point[0][0])

                    # ✅ 선속도 및 각속도 결정
                    out_vel = self.status_msg.twist.linear.x
                    out_rad_vel = 2 * out_vel * sin(theta) / self.lfd       

                    self.cmd_msg.linear.x=out_vel
                    self.cmd_msg.angular.z=out_rad_vel
            else :
                print("⚠️ 전방 주시 포인트를 찾을 수 없음")
                self.cmd_msg.linear.x=0.0
                self.cmd_msg.angular.z=0.0
            
            self.cmd_pub.publish(self.cmd_msg)

    def odom_callback(self, msg):
        """ Odometry 데이터를 받아 로봇의 현재 위치와 방향 저장 """
        self.is_odom=True
        self.odom_msg=msg
        q = self.odom_msg.pose.pose.orientation

        _, _, self.robot_yaw = Quaternion(q.w, q.x, q.y, q.z).to_euler()
    
    def path_callback(self, msg):
        """ 지역 경로 데이터를 수신하여 저장 """
        self.is_path=True
        self.path_msg=msg

    def status_callback(self,msg):
        """ 로봇 상태 데이터를 받아 속도 정보를 저장 """
        self.is_status=True
        self.status_msg=msg
        
def main(args=None):
    """ 노드 실행 함수 """
    rclpy.init(args=args)
    path_tracker = followTheCarrot()
    rclpy.spin(path_tracker)

    path_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()