import React, { useEffect } from 'react';
import { BrowserRouter } from 'react-router-dom';
import AppRoutes from './routes/AppRoutes';
import { useAuthStore } from './stores/authStore';

function App() {
  const { isTokenValid, refreshAccessToken, clearToken } = useAuthStore();

  // 앱 초기화 시 토큰 유효성 검사 및 리프레시
  useEffect(() => {
    const initializeAuth = async () => {
      // 토큰이 유효한지 확인
      if (!isTokenValid()) {
        console.log('토큰이 유효하지 않거나 만료되었습니다. 리프레시 시도...');
        
        try {
          // 토큰 리프레시 시도
          const refreshed = await refreshAccessToken();
          
          if (refreshed) {
            console.log('토큰 리프레시 성공');
          } else {
            console.log('토큰 리프레시 실패, 로그아웃 처리');
            clearToken();
          }
        } catch (error) {
          console.error('토큰 리프레시 중 오류 발생:', error);
          clearToken();
        }
      } else {
        console.log('유효한 토큰이 있습니다.');
      }
    };

    initializeAuth();
  }, [isTokenValid, refreshAccessToken, clearToken]);

  return (
    <BrowserRouter>
      <AppRoutes />
    </BrowserRouter>
  );
}

export default App;
