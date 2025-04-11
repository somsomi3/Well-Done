import React, { useEffect, useState } from 'react';
import { BrowserRouter, useLocation } from 'react-router-dom';
import AppRoutes from './routes/AppRoutes';
import { useAuthStore } from './stores/authStore';
import { getTokenRefreshInterval, getTokenExpiryThreshold } from './configs/env';
import ErrorBoundary from './components/ErrorBoundary';
import Toast from './components/atoms/Toast/Toast';
import { useToastStore } from './stores/toastStore';

// 인증이 필요하지 않은 경로 목록
const PUBLIC_PATHS = ['/login', '/register', '/forgot-password'];

// 경로 확인을 위한 래퍼 컴포넌트
const AppContent = () => {
  const location = useLocation();
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
  
  // 현재 경로가 인증이 필요하지 않은 경로인지 확인
  const isPublicPath = PUBLIC_PATHS.includes(location.pathname);

  // 초기 인증 상태 확인 및 토큰 리프레시
  useEffect(() => {
    const initializeAuthProcess = async () => {
      // 로컬 스토리지에서 토큰 복원 시도
      const tokenRestored = initializeAuth();
      
      try {
        // 공개 경로에서는 토큰 검증만 하고 갱신은 시도하지 않음
        if (isPublicPath) {
          console.log('공개 페이지에서는 토큰 갱신을 시도하지 않습니다.');
          setIsInitializing(false);
          return;
        }
        
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

    initializeAuthProcess();
  }, [isTokenValid, refreshAccessToken, clearToken, initializeAuth, isPublicPath]);

  // 주기적으로 토큰 만료 시간 체크 및 필요 시 리프레시
  useEffect(() => {
    if (isInitializing || isPublicPath) return;

    const tokenExpiryThreshold = getTokenExpiryThreshold();
    const refreshInterval = getTokenRefreshInterval();
    
    console.log(`토큰 만료 임계값: ${tokenExpiryThreshold}초, 체크 간격: ${refreshInterval / 1000}초`);
    
    const checkTokenStatus = async () => {
      // 이미 리프레시 중이면 중복 실행 방지
      if (isRefreshing) {
        console.log('이미 토큰 리프레시가 진행 중입니다.');
        return;
      }
      
      if (!isTokenValid()) {
        console.log('토큰이 만료되었습니다. 리프레시 시도...');
        await refreshAccessToken();
        return;
      }
      
      // 토큰 만료까지 남은 시간 확인
      const expiryTime = getTokenExpiryTime();
      console.log(`토큰 만료까지 남은 시간: ${expiryTime}초`);
      
      // 만료 임계값보다 적게 남았으면 미리 리프레시
      if (expiryTime <= tokenExpiryThreshold) {
        console.log('토큰 만료가 임박했습니다. 미리 리프레시 시도...');
        await refreshAccessToken();
      }
    };
    
    // 초기 체크
    checkTokenStatus();
    
    // 주기적으로 체크
    const intervalId = setInterval(checkTokenStatus, refreshInterval);
    
    // 컴포넌트 언마운트 시 인터벌 정리
    return () => clearInterval(intervalId);
  }, [
    isInitializing, 
    isTokenValid, 
    refreshAccessToken, 
    getTokenExpiryTime,
    isRefreshing,
    isPublicPath
  ]);

  // 초기화 중에는 로딩 표시
  if (isInitializing) {
    return <div className="flex items-center justify-center h-screen">로딩 중...</div>;
  }

  return (
    <>
      <AppRoutes />
      {/* 토스트 알림 컴포넌트 */}
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
    </>
  );
};

function App() {
  return (
    <ErrorBoundary>
      <BrowserRouter>
        <AppContent />
      </BrowserRouter>
    </ErrorBoundary>
  );
}

export default App;
