// 환경 변수에서 API URL을 가져오는 함수
export const getApiUrl = () => {
  // Vite 환경 변수 사용
  return import.meta.env.VITE_BASE_URL || '/api';
};

// 환경 변수에서 다른 설정값들을 가져오는 함수들
export const getTokenRefreshInterval = () => {
  return parseInt(import.meta.env.VITE_TOKEN_REFRESH_INTERVAL || '300000', 10); // 기본값 5분
};

export const getTokenExpiryThreshold = () => {
  return parseInt(import.meta.env.VITE_TOKEN_EXPIRY_THRESHOLD || '60', 10); // 기본값 60초
};

// 기타 필요한 환경 설정 값들을 가져오는 함수들...
