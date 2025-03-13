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

## 🏃‍♂️ 작성한 코드 실행

### 1️⃣ ROS2 실행 환경 설정

ROS2 패키지를 실행하기 전에, **ROS2 환경을 설정하는 명령어**를 실행해야 합니다.
**개발자의 환경에 따라 경로가 다를 수 있으므로, 본인의 환경에 맞게 수정해야 합니다.**

```bash
# ROS2 실행을 위한 환경 변수 설정
call C:\dev\ros2_eloquent\setup.bat

# 워크스페이스에서 설치된 패키지를 사용하도록 설정
call C:\Users\SSAFY\Desktop\temp\S12P21E102\sim\ros2_ws\install\local_setup.bat
```

**💡 주의**<br>
위의 환경 변수 설정은 **새로운 터미널을 열 때마다 실행해야 합니다.**

### 2️⃣ 워크스페이스 내 특정 패키지 실행하기

ROS2 패키지에서 작성된 노드를 실행하려면 아래 명령어를 사용합니다.

```bash
# 특정 패키지의 노드 실행
ros2 run [패키지 이름] [노드 이름]
```

**✅ 예시**

- `sub1` 패키지의 `publisher` 노드를 실행

```bash
ros2 run sub1 publisher
```

- `sub1` 패키지의 `subscriber` 노드를 실행

```bash
ros2 run sub1 subscriber
```

### 3️⃣ RQT를 통해 토픽 데이터 확인하기

ROS2에서는 `rqt`를 사용하여 **토픽 데이터를 시각적으로 모니터링**할 수 있습니다.

```bash
# RQT 실행
rqt
```

실행 후, `rqt`에서 `Plugins → Topics → Topic Monitor`를 선택하면
현재 실행 중인 **토픽 데이터를 실시간으로 확인**할 수 있습니다.

## 🔗 시뮬레이터 데이터를 ROS로 전달하기 (ssafy_bridge 실행)

시뮬레이터에서 생성된 데이터를 ROS2로 전달하기 위해 `ssafy_bridge`를 실행해야 합니다.

### 1️⃣ ROS2 환경 설정

ssafy_bridge 실행 전에, ROS2와 관련된 환경을 설정해야 합니다.  
아래 명령어를 실행하여 ROS2 패키지와 워크스페이스의 환경을 적용합니다.

```bash
# ROS2 실행을 위한 환경 변수 설정
call C:\dev\ros2_eloquent\setup.bat

# 워크스페이스 내 패키지를 사용할 수 있도록 환경 변수 설정
call C:\Users\SSAFY\Desktop\temp\S12P21E102\sim\ros2_ws\install\local_setup.bat
```

⚠️ 중요:
새로운 터미널을 실행할 때마다 위 명령어를 입력하여 환경을 설정해야 합니다.

### 2️⃣ ssafy_bridge 실행

ssafy_bridge를 실행하여 시뮬레이터 데이터를 ROS2로 전달하는 과정입니다.

1. ssafy_bridge가 있는 디렉토리로 이동합니다.

```bash
cd C:\Users\SSAFY\Desktop\catkin_ws\src\ros2_smart_home\ssafy_bridge\launch
```

2. ros2 launch 명령어를 사용하여 ssafy_bridge를 실행합니다.

```bash
ros2 launch ssafybridge_launch.py
```

### 3️⃣ 실행 로그 확인

정상적으로 실행되면 아래와 같은 로그가 출력됩니다.

```bash
[INFO] [Launch]: Default logging verbosity is set to INFO
[INFO] [udp_to_pub.EXE-1]: process started with pid [15340]
[INFO] [sub_to_udp.EXE-2]: process started with pid [7988]
[INFO] [udp_to_cam.EXE-3]: process started with pid [5864]
[INFO] [udp_to_laser.EXE-4]: process started with pid [9592]
```

### 4️⃣ 추가 참고 사항

- ssafy_bridge 실행을 위한 launch.py 파일은 ros2_smart_home/ssafy_bridge/launch 폴더에 위치합니다.
- 실행 도중 에러가 발생할 경우, ROS2 환경 설정 (setup.bat 및 local_setup.bat)이 올바르게 실행되었는지 확인하세요.
- 새로운 터미널을 실행할 때마다 ROS2 환경 설정을 다시 해야 합니다.
