import rclpy
from rclpy.node import Node
import sys
import time

# Windows에서는 msvcrt.kbhit()를 사용
try:
    import msvcrt
    IS_WINDOWS = True
except ImportError:
    import select
    IS_WINDOWS = False

from ssafy_msgs.msg import TurtlebotStatus,HandControl

# 수신 데이터 : 터틀봇 상태 (/turtlebot_status)
# 송신 데이터 : Hand Control 제어 (/hand_control)

class Handcontrol(Node):

    def __init__(self):
        super().__init__('hand_control')
                
        ## ✅ 1. Publisher & Subscriber 생성
        self.hand_control = self.create_publisher(HandControl, '/hand_control', 10)                
        self.turtlebot_status = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.turtlebot_status_cb,10)
        
        ## ✅ 2. 제어 메시지 변수 생성
        self.hand_control_msg=HandControl()        
        self.turtlebot_status_msg = TurtlebotStatus()
        self.is_turtlebot_status = False

    def display_status(self):
        """✅ 현재 터틀봇 상태를 실시간으로 출력"""
        if self.is_turtlebot_status:
            print("\n🔍 현재 터틀봇 상태:")
            print(f"   - can_lift: {'✅ 가능' if self.turtlebot_status_msg.can_lift else '❌ 불가능'}")
            print(f"   - can_put: {'✅ 가능' if self.turtlebot_status_msg.can_put else '❌ 불가능'}")

    def run(self):
        """🚀 인터럽트 방식으로 상태 갱신 + 입력 감지"""
        print("✅ Hand Control 노드 실행 중... (CTRL+C로 종료)")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)  # 상태 갱신
            self.display_status()

            # 🔍 사용자의 입력 대기 (인터럽트 방식)
            print("\n📌 Select Menu [0: status_check, 1: preview, 2: pick_up, 3: put_down] (아무 키도 입력하지 않으면 상태 갱신)")
            sys.stdout.flush()

            if self.check_input_ready():
                menu = input(">> ").strip()
                self.handle_input(menu)

    def check_input_ready(self):
        """🎯 사용자가 입력을 했는지 감지하는 함수"""
        if IS_WINDOWS:
            return msvcrt.kbhit()
        return select.select([sys.stdin], [], [], 0.1)[0]

    def handle_input(self, menu):
        """✅ 사용자의 입력을 처리하는 함수"""
        if menu == '0':               
            self.hand_control_status()
        elif menu == '1':
            self.hand_control_preview()               
        elif menu == '2':
            self.hand_control_pick_up()   
        elif menu == '3':
            self.hand_control_put_down()
        else:
            print("❌ 잘못된 입력입니다. 다시 선택해주세요.")

    def hand_control_status(self):
        """✅ 현재 터틀봇 상태 출력"""
        self.display_status()

    def hand_control_preview(self):
        """✅ Preview 모드 실행"""
        print("📸 Hand Control - Preview 모드 실행")
        self.hand_control_msg.control_mode = 1
        self.hand_control.publish(self.hand_control_msg)

    def hand_control_pick_up(self):
        """✅ Pick Up 실행"""
        if not self.is_turtlebot_status:
            print("❌ 아직 터틀봇 상태 정보를 수신하지 못했습니다.")
            return

        if self.turtlebot_status_msg.can_lift:
            print("🤖 오브젝트를 집습니다.")
            self.hand_control_msg.control_mode = 2
            self.hand_control.publish(self.hand_control_msg)
        else:
            print("❌ 현재 오브젝트를 들 수 없습니다.")

        
    def hand_control_put_down(self):
        """✅ Put Down 실행"""
        if not self.is_turtlebot_status:
            print("❌ 아직 터틀봇 상태 정보를 수신하지 못했습니다.")
            return

        if self.turtlebot_status_msg.can_put:
            try:
                put_distance = float(input("📏 오브젝트를 놓을 거리 (m): "))
                put_height = float(input("📐 오브젝트를 놓을 높이 (m): "))

                print(f"🛠️ 오브젝트를 놓습니다. (거리: {put_distance}m, 높이: {put_height}m)")
                self.hand_control_msg.control_mode = 3
                self.hand_control_msg.put_distance = put_distance
                self.hand_control_msg.put_height = put_height
                self.hand_control.publish(self.hand_control_msg)
            except ValueError:
                print("❌ 잘못된 입력입니다. 숫자를 입력해주세요.")
        else:
            print("❌ 현재 오브젝트를 내려놓을 수 없습니다.")

    def turtlebot_status_cb(self, msg):
        """✅ 터틀봇 상태 업데이트"""
        self.is_turtlebot_status = True
        self.turtlebot_status_msg = msg
        

def main(args=None):
    """🌟 메인 실행 함수"""
    rclpy.init(args=args)
    sub1_hand_control = Handcontrol()    

    try:
        sub1_hand_control.run()
    except KeyboardInterrupt:
        print("🛑 프로그램 종료 중...")
    finally:
        sub1_hand_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()