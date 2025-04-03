import React, { useEffect } from 'react';
import { BrowserRouter } from 'react-router-dom';
import AppRoutes from './routes/AppRoutes';
import { useAuthStore } from './stores/authStore';
import { jwtDecode } from 'jwt-decode';

function App() {
  const { token, setToken, logout } = useAuthStore();

  // 토큰 유효성 검사 및 초기화
  useEffect(() => {
    const storedToken = localStorage.getItem('accessToken');
    
    if (storedToken) {
      try {
        // 토큰 디코딩하여 만료 여부 확인
        const decodedToken = jwtDecode(storedToken);
        const currentTime = Date.now() / 1000;
        
        if (decodedToken.exp < currentTime) {
          // 토큰이 만료된 경우
          console.log('토큰이 만료되었습니다. 로그아웃 처리합니다.');
          logout();
        } else {
          // 유효한 토큰인 경우 상태 업데이트
          setToken(storedToken);
        }
      } catch (error) {
        // 토큰 디코딩 실패 시 (유효하지 않은 토큰)
        console.error('유효하지 않은 토큰:', error);
        logout();
      }
    }
  }, [setToken, logout]);

  return (
    <BrowserRouter>
      <AppRoutes />
    </BrowserRouter>
  );
}

export default App;
