import React, { useEffect, useState } from 'react';
import { BrowserRouter } from 'react-router-dom';
import AppRoutes from './routes/AppRoutes';
import { useAuthStore } from './stores/authStore';
import { getTokenRefreshInterval, getTokenExpiryThreshold } from './configs/env';
import ErrorBoundary from './components/ErrorBoundary';
import Toast from './components/atoms/Toast/Toast';
import { useToastStore } from './stores/toastStore';

function App() {
  const { 
    isTokenValid, 
    refreshAccessToken, 
    clearToken, 
    getTokenExpiryTime,
    isRefreshing,
    initializeAuth
  } = useAuthStore();
  const { toasts } = useToastStore();
  const [isInitializing, setIsInitializing] = useState(true);

  useEffect(() => {
    const initializeAuth2 = () => {
      // initializeAuth2 함수의 내용
    };

    const initializeAuthProcess = async () => {
      initializeAuth(); // 로컬 스토리지에서 토큰 복원 시도
      
      try {
        if (!isTokenValid()) {
          console.log('토큰이 유효하지 않거나 만료되었습니다. 리프레시 시도...');
          
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

    initializeAuthProcess();
  }, [isTokenValid, refreshAccessToken, clearToken, initializeAuth]);

  useEffect(() => {
    if (isInitializing) return;

    const tokenExpiryThreshold = getTokenExpiryThreshold();
    const refreshInterval = getTokenRefreshInterval();
    
    console.log(`토큰 만료 임계값: ${tokenExpiryThreshold}초, 체크 간격: ${refreshInterval / 1000}초`);
    
    const checkTokenStatus = async () => {
      if (isRefreshing) {
        console.log('이미 토큰 리프레시가 진행 중입니다.');
        return;
      }
      
      if (!isTokenValid()) {
        console.log('토큰이 만료되었습니다. 리프레시 시도...');
        await refreshAccessToken();
        return;
      }
      
      const expiryTime = getTokenExpiryTime();
      console.log(`토큰 만료까지 남은 시간: ${expiryTime}초`);
      
      if (expiryTime <= tokenExpiryThreshold) {
        console.log('토큰 만료가 임박했습니다. 미리 리프레시 시도...');
        await refreshAccessToken();
      }
    };
    
    checkTokenStatus();
    
    const intervalId = setInterval(checkTokenStatus, refreshInterval);
    
    return () => clearInterval(intervalId);
  }, [
    isInitializing, 
    isTokenValid, 
    refreshAccessToken, 
    getTokenExpiryTime,
    isRefreshing
  ]);

  if (isInitializing) {
    return <div className="flex items-center justify-center h-screen">로딩 중...</div>;
  }

  return (
    <ErrorBoundary>
      <BrowserRouter>
        <AppRoutes />
        <div className="toast-container fixed bottom-4 right-4 z-50">
          {toasts.map(toast => (
            <Toast 
              key={toast.id} 
              message={toast.message} 
              type={toast.type} 
              id={toast.id} 
            />
          ))}
        </div>
      </BrowserRouter>
    </ErrorBoundary>
  );
}

export default App;
