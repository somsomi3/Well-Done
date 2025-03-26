import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,Point
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt,atan2
import numpy as np

# path_tracking 노드는 로봇의 위치(/odom), 로봇의 속도(/turtlebot_status), 주행 경로(/local_path)를 받아서, 주어진 경로를 따라가게 하는 제어 입력값(/cmd_vel)을 계산합니다.
# 제어입력값은 선속도와 각속도로 두가지를 구합니다. 
# sub2의 path_tracking은 sub1의 path_tracking를 사용해도 됩니다.


# 노드 로직 순서
# 1. 제어 주기 및 타이머 설정
# 2. 파라미터 설정
# 3. Quaternion 을 euler angle 로 변환
# 4. 터틀봇이 주어진 경로점과 떨어진 거리(lateral_error)와 터틀봇의 선속도를 이용해 전방주시거리 계산
# 5. 전방 주시 포인트 설정
# 6. 전방 주시 포인트와 로봇 헤딩과의 각도 계산
# 7. 선속도, 각속도 정하기


class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        self.path_sub = self.create_subscription(Path,'/local_path',self.path_callback,10)

        # 로직 1. 제어 주기 및 타이머 설정
        time_period=0.01
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom=False
        self.is_path=False
        self.is_status=False

        self.odom_msg=Odometry()            
        self.robot_yaw=0.0
        self.path_msg=Path()
        self.cmd_msg=Twist()

        # 로직 2. 파라미터 설정
        self.lfd=0.1
        self.min_lfd=0.1
        self.max_lfd=2.0


    def timer_callback(self):

        if self.is_status and self.is_odom ==True and self.is_path==True:


            if len(self.path_msg.poses)> 1:
                self.is_look_forward_point= False
                
                # 로봇의 현재 위치를 나타내는 변수
                robot_pose_x=self.odom_msg.pose.pose.position.x
                robot_pose_y=self.odom_msg.pose.pose.position.y

                # 로봇이 경로에서 떨어진 거리를 나타내는 변수
                lateral_error= sqrt(
                    pow(self.path_msg.poses[0].pose.position.x-robot_pose_x,2)+
                    pow(self.path_msg.poses[0].pose.position.y-robot_pose_y,2)
                )

                print(robot_pose_x,robot_pose_y,lateral_error)
                
                # self.lfd = lateral_error * 1.5  # 예시 비례 상수
                # if self.lfd < self.min_lfd:
                #     self.lfd = self.min_lfd
                # if self.lfd > self.max_lfd:
                #     self.lfd = self.max_lfd
                self.lfd = 0.5 # 고정된 전방 주시 거리 사용
                min_dis=float('inf')

                for num, waypoint in enumerate(self.path_msg.poses):
                    self.current_point = waypoint.pose.position
                    dis = sqrt(
                        pow(self.current_point.x - robot_pose_x, 2) +
                        pow(self.current_point.y - robot_pose_y, 2)
                    )
                    if abs(dis - self.lfd) < min_dis:
                        min_dis = abs(dis - self.lfd)
                        self.forward_point = self.current_point
                        self.is_look_forward_point = True             
                
                if self.is_look_forward_point :
            
                    global_forward_point=[self.forward_point.x ,self.forward_point.y,1]

                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                        [sin(self.robot_yaw),  cos(self.robot_yaw), robot_pose_y],
                        [0,                   0,                   1]
                    ])
                    det_trans_matrix = np.linalg.inv(trans_matrix)
                    local_forward_point = det_trans_matrix.dot(np.array(global_forward_point).reshape(3, 1))

                    theta = -atan2(local_forward_point[1][0], local_forward_point[0][0])
                    
                    out_vel = max(0.0, 1.0 * cos(theta))  # 전진 속도
                    Kp = 1.5
                    out_rad_vel = Kp * theta  # 조향 각속도         

                    # 제한 걸기
                    out_rad_vel = max(-1.0, min(1.0, out_rad_vel))

                    self.cmd_msg.linear.x = float(out_vel)
                    self.cmd_msg.angular.z = float(out_rad_vel)                 
                else:
                    print("⚠️ 전방 주시 포인트를 찾을 수 없음")
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0

            else :
                print("⚠️ 경로가 충분하지 않음")
                self.cmd_msg.linear.x=0.0
                self.cmd_msg.angular.z=0.0
            
            self.cmd_pub.publish(self.cmd_msg)
            
    def odom_callback(self, msg):
        self.is_odom=True
        self.odom_msg=msg
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = Quaternion(q.w, q.x, q.y, q.z).to_euler()

    
    def path_callback(self, msg):
        self.is_path=True
        self.path_msg=msg


    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg
        

        
def main(args=None):
    rclpy.init(args=args)
    node = followTheCarrot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()