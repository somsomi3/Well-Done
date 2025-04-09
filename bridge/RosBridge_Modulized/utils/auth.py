# auth.py
import requests

def get_jwt_token(node):
    """Spring 서버에서 JWT 토큰 획득"""
    try:
        # 로그인 요청
        login_data = {
            "username": "robot",
            "password": "robotrobot",
        }
        
        # 타임아웃을 짧게 설정하여 연결 시도
        response = requests.post(
            f"{node.spring_server_url}/api/auth/login",
            json=login_data,
            timeout=2.0  # 2초 타임아웃
        )
        
        if response.status_code == 200:
            response_data = response.json()
            token = response_data.get("accessToken")
            
            if token:
                node.get_logger().info("JWT 토큰 획득 성공")
                return token
            else:
                node.get_logger().warning("응답에서 토큰을 찾을 수 없습니다")
                return None
        else:
            node.get_logger().warning(
                f"JWT 토큰 획득 실패: {response.status_code}"
            )
            return None
    except requests.exceptions.ConnectionError:
        node.get_logger().warning(
            "Spring 서버에 연결할 수 없습니다. 토큰 없이 계속 진행합니다."
        )
        return None
    except Exception as e:
        node.get_logger().warning(f"JWT 토큰 획득 중 오류: {str(e)}")
        return None

def refresh_token(node):
    """만료된 토큰을 새로고침"""
    node.get_logger().info("JWT 토큰 새로고침 시도")
    new_token = get_jwt_token(node)
    if new_token:
        node.jwt_token = new_token
        return True
    return False