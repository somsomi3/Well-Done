import axios from 'axios';
import { useAuthStore } from '../stores/authStore';
import { getApiUrl } from '../config/env';

// API 기본 URL 설정 - 환경 변수에서 가져옴
const baseURL = getApiUrl();

// 인증이 필요한 요청을 위한 인스턴스
const api = axios.create({
  baseURL,
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  }
});

// 인증이 필요없는 요청을 위한 인스턴스
const publicApi = axios.create({
  baseURL,
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  }
});

// 요청 인터셉터 - 인증이 필요한 요청에 토큰 추가
api.interceptors.request.use(
  (config) => {
    const { token } = useAuthStore.getState();
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

// 응답 인터셉터 - 토큰 만료 시 리프레시 처리
api.interceptors.response.use(
  (response) => response,
  async (error) => {
    const originalRequest = error.config;
    
    // 401 에러이고 아직 재시도하지 않은 경우에만 리프레시 시도
    if (error.response?.status === 401 && !originalRequest._retry) {
      originalRequest._retry = true;
      
      const { 
        refreshAccessToken, 
        clearToken, 
        isRefreshing 
      } = useAuthStore.getState();
      
      // 이미 리프레시 중이면 대기
      if (isRefreshing) {
        try {
          // 리프레시가 완료될 때까지 잠시 대기
          await new Promise(resolve => setTimeout(resolve, 1000));
          
          // 토큰이 갱신되었는지 확인
          const { token, isRefreshing: stillRefreshing } = useAuthStore.getState();
          
          if (token && !stillRefreshing) {
            // 토큰이 갱신되었으면 원래 요청 재시도
            originalRequest.headers.Authorization = `Bearer ${token}`;
            return api(originalRequest);
          } else {
            // 리프레시가 실패했거나 여전히 진행 중이면 로그인 페이지로 이동
            clearToken();
            window.location.href = '/login';
            return Promise.reject(error);
          }
        } catch (waitError) {
          clearToken();
          window.location.href = '/login';
          return Promise.reject(waitError);
        }
      }
      
      try {
        const refreshed = await refreshAccessToken();
        if (refreshed) {
          const { token } = useAuthStore.getState();
          originalRequest.headers.Authorization = `Bearer ${token}`;
          return api(originalRequest);
        } else {
          // 리프레시 실패 시 로그아웃 처리
          clearToken();
          // 로그인 페이지로 리다이렉트
          window.location.href = '/login';
          return Promise.reject(error);
        }
      } catch (refreshError) {
        // 리프레시 과정에서 오류 발생 시 로그아웃 처리
        clearToken();
        // 로그인 페이지로 리다이렉트
        window.location.href = '/login';
        return Promise.reject(refreshError);
      }
    }
    
    // 다른 종류의 에러는 그대로 반환
    return Promise.reject(error);
  }
);

// 공개 API 요청에 대한 인터셉터 설정
publicApi.interceptors.request.use(
  (config) => config,
  (error) => Promise.reject(error)
);

publicApi.interceptors.response.use(
  (response) => response,
  (error) => Promise.reject(error)
);

export { api, publicApi };