# ROS2 Workspace (ros2_ws)

이 폴더는 **ROS2 패키지**를 관리하는 워크스페이스입니다.

## 🚀 빌드하는 방법

### 1️⃣ Native Tools Command Prompt for VS 2022 실행

1. **관리자 권한으로** `Native Tools Command Prompt for VS 2022`를 실행합니다.
2. 아래 명령어를 차례로 입력합니다.

```bash
# ROS2 명령어 실행을 위한 배치 파일 실행
call C:\dev\ros2_eloquent\setup.bat

# ros2_ws 폴더로 이동 (경로는 사용자의 환경에 맞게 변경)
cd C:\Users\SSAFY\Desktop\Project\S12P21E102\sim\ros2_ws

# 전체 패키지 빌드 실행
colcon build

# 특정 패키지만 빌드하려면 --packages-select 옵션 사용
colcon build --packages-select [패키지 이름]

# 예시: sub1 패키지만 빌드하려면
colcon build --packages-select sub1
```
