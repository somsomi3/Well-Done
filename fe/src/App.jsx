import React, { useEffect, useState } from 'react';
import { BrowserRouter } from 'react-router-dom';
import AppRoutes from './routes/AppRoutes';
import { useAuthStore } from './features/auth/store/authStore';

function App() {
  const { isTokenValid, refreshAccessToken, clearToken } = useAuthStore();
  const [isInitializing, setIsInitializing] = useState(true);

  useEffect(() => {
    const initializeAuth = async () => {
      try {
        if (!isTokenValid()) {
          console.log('토큰이 유효하지 않거나 만료되었습니다. 리프레시 시도...');
          
          // 최대 1회만 리프레시 시도
          const refreshed = await refreshAccessToken();
          
          if (refreshed) {
            console.log('토큰 리프레시 성공');
          } else {
            console.log('토큰 리프레시 실패, 로그아웃 처리');
            clearToken();
          }
        } else {
          console.log('유효한 토큰이 있습니다.');
        }
      } catch (error) {
        console.error('인증 초기화 중 오류 발생:', error);
        clearToken();
      } finally {
        setIsInitializing(false);
      }
    };

    initializeAuth();
  }, [isTokenValid, refreshAccessToken, clearToken]);

  // 초기화 중에는 로딩 표시
  if (isInitializing) {
    return <div className="flex items-center justify-center h-screen">로딩 중...</div>;
  }

  return (
    <BrowserRouter>
      <AppRoutes />
    </BrowserRouter>
  );
}

export default App;
