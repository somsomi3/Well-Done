import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus,EnviromentStatus
from std_msgs.msg import Float32,Int8MultiArray

# controller는 시뮬레이터로 부터를 데이터를 수신해서 확인(출력)하고, 송신해서 제어가 되는지 확인해보는 통신 테스트를 위한 노드입니다.
# 메시지를 받아서 어떤 데이터들이 있는지 확인하고, 어떤 메시지를 보내야 가전 또는 터틀봇이 제어가 되는지 확인해보면서 ros2 통신에 익숙해지세요.
# 수신 데이터 : 터틀봇 상태(/turtlebot_status), 환경정보(/envir_status), 가전정보(/app_status)
# 송신 데이터 : 터틀봇 제어(/ctrl_cmd), 가전제어(/app_control)

# 노드 로직 순서
# 1. 수신 데이터 출력
# 2. 특정 가전제품 ON
# 3. 특정 가전제품 OFF
# 4. 터틀봇 정지
# 5. 터틀봇 시계방향 회전
# 6. 터틀봇 반시계방향 회전


class Controller(Node):

    def __init__(self):
        super().__init__('sub1_controller')
        ## 메시지 송신을 위한 PUBLISHER 생성
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.app_control_pub = self.create_publisher(Int8MultiArray, 'app_control', 10)
        ## 메시지 수신을 위한 SUBSCRIBER 생성
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.envir_status_sub = self.create_subscription(EnviromentStatus,'/envir_status',self.envir_callback,10)
        self.app_status_sub = self.create_subscription(Int8MultiArray,'/app_status',self.app_callback,10)
        
        ## 타이머 생성
        self.timer = self.create_timer(1, self.timer_callback)

        ## 제어 메시지 변수 생성 -> 타입 설정
        ### 터틀봇의 이동 명령을 담는 메시지 객체
        ### Twist는 ROS2의 메시지 타입으로, 선속도(linear)와 각속도(angular) 정보를 담음
        self.cmd_msg=Twist()
        
        ### 가전 제품을 제어할 메시지를 담는 객체
        ### Int8MultiArray는 정수 배열을 저장할 수 있는 메시지 타입
        self.app_control_msg=Int8MultiArray()
        for i in range(17):
            self.app_control_msg.data.append(0)

        ### 터틀봇의 현재 상태 정보를 저장하는 객체
        ### /turtlebot_status 토픽에서 수신한 데이터를 담음
        self.turtlebot_status_msg=TurtlebotStatus()
        ### 환경 정보를 저장하는 객체
        ### /envir_status 토픽에서 받아온 데이터를 담음
        self.envir_status_msg=EnviromentStatus()
        ### 가전제품 상태 정보를 저장하는 객체
        ### /app_status 토픽에서 받아온 데이터를 담음
        self.app_status_msg=Int8MultiArray()

        ### 데이터가 들어왔는지 여부를 추적하는 역할
        self.is_turtlebot_status=False
        self.is_app_status=False
        self.is_envir_status=False

    ## 터틀봇 상태를 받아서 저장
    def listener_callback(self, msg):
        self.is_turtlebot_status=True
        self.turtlebot_status_msg=msg
    ## 환경 데이터를 받아서 저장
    def envir_callback(self, msg):
        self.is_envir_status=True
        self.envir_status_msg=msg
    ## 가전제품 상태 데이터를 받아서 저장
    def app_callback(self, msg):
        self.is_app_status=True
        self.app_status_msg=msg  

    ## 모든 가전제품을 켬
    def app_all_on(self):
        for i in range(17):
            self.app_control_msg.data[i]=1
        self.app_control_pub.publish(self.app_control_msg)
    ## 모든 가전제품을 끔
    def app_all_off(self):
        for i in range(17):
            self.app_control_msg.data[i]=2
        self.app_control_pub.publish(self.app_control_msg)
    ## 특정 가전제품을 켬
    def app_select_on(self,num):
        if 0 <= num < 17:
            self.app_control_msg.data[num] = 1 # 가전제품[num] on
            self.app_control_pub.publish(self.app_control_msg)
        else:
            self.get_logger().warn(f"Invalid appliance number: {num}")
    ## 특정 가전제품을 끔
    def app_select_off(self,num):
        if 0 <= num < 17:
            self.app_control_msg.data[num] = 2 # 가전제품[num] off
            self.app_control_pub.publish(self.app_control_msg)
        else:
            self.get_logger().warn(f"Invalid appliance number: {num}")

    ## 터틀봇 전진
    def turtlebot_go(self) :
        self.cmd_msg.linear.x=0.5
        self.cmd_msg.angular.z=0.0
    ## 터틀봇 정지
    def turtlebot_stop(self) :
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.0
        self.cmd_publisher.publish(self.cmd_msg)
    ## 터틀봇 시계방향 회전
    def turtlebot_cw_rot(self) :
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = -0.5 # 시계방향 회전
        self.cmd_publisher.publish(self.cmd_msg)
    ## 터틀봇 반시계방향 회전
    def turtlebot_ccw_rot(self) :
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.5  # 반시계방향 회전
        self.cmd_publisher.publish(self.cmd_msg)

    ## 주기적으로 실행되는 함수
    def timer_callback(self):
        ## 터틀봇 상태 출력
        if self.is_turtlebot_status:
            self.get_logger().info(
                f"current_linear_vel : {self.turtlebot_status_msg.twist.linear.x}, "
                f"current_angular_vel : {self.turtlebot_status_msg.twist.angular.z}, "
                f"battery : {self.turtlebot_status_msg.battery_percentage}, "
                f"Charging: {self.turtlebot_status_msg.power_supply_status}"
            )
        ## 환경 상태 출력
        if self.is_envir_status:
            self.get_logger().info(
                f"date : {self.envir_status_msg.month}/{self.envir_status_msg.day} {self.envir_status_msg.hour}:{self.envir_status_msg.minute}, "
                f"time : {self.envir_status_msg.hour}:{self.envir_status_msg.minute}, "
                f"temp : {self.envir_status_msg.temperature}, "
                f"weather : {self.envir_status_msg.weather}"
            )
        ## 가전제품 상태 출력
        if self.is_app_status:
            self.get_logger().info(f"Appliance Status: {self.app_status_msg.data}")

        ## 테스트 할려면 하나씩 주석 제거 하시오
        ## IOT(가전) 제어 함수
        # self.app_all_on()
        # self.app_all_off()
        # self.app_select_on(12)
        # self.app_select_off(16)


        ## 터틀봇 제어 함수
        # self.turtlebot_go()
        # self.turtlebot_stop()
        # self.turtlebot_cw_rot()
        # self.turtlebot_ccw_rot()

        self.cmd_publisher.publish(self.cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    sub1_controller = Controller()
    rclpy.spin(sub1_controller)
    sub1_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()