import React, { useEffect, useState } from 'react';
import { BrowserRouter } from 'react-router-dom';
import AppRoutes from './routes/AppRoutes';
import { useAuthStore } from './stores/authStore';
import { getTokenRefreshInterval, getTokenExpiryThreshold } from './configs/env';

function App() {
  const { isTokenValid, refreshAccessToken, clearToken, getTokenExpiryTime } =
    useAuthStore();
  const [isInitializing, setIsInitializing] = useState(true);

  // 초기 인증 상태 확인 및 토큰 리프레시
  useEffect(() => {
    const initializeAuth = async () => {
      try {
        if (!isTokenValid()) {
          console.log(
            '토큰이 유효하지 않거나 만료되었습니다. 리프레시 시도...'
          );

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

  // 주기적으로 토큰 만료 시간 체크 및 필요 시 리프레시
  useEffect(() => {
    if (isInitializing) return;

    const tokenExpiryThreshold = getTokenExpiryThreshold(); // 토큰 만료 임계값 (초)
    const refreshInterval = getTokenRefreshInterval(); // 토큰 체크 간격 (밀리초)

    console.log(
      `토큰 만료 임계값: ${tokenExpiryThreshold}초, 체크 간격: ${
        refreshInterval / 1000
      }초`
    );

    // 토큰 상태 체크 함수
    const checkTokenStatus = async () => {
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
  }, [isInitializing, isTokenValid, refreshAccessToken, getTokenExpiryTime]);

  // 초기화 중에는 로딩 표시
  if (isInitializing) {
    return (
      <div className="flex items-center justify-center h-screen">
        로딩 중...
      </div>
    );
  }

  return (
    <BrowserRouter>
      <AppRoutes />
    </BrowserRouter>
  );
}

export default App;
