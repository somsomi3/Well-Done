import axios from 'axios';
import { useAuthStore } from '../stores/authStore';

// API 기본 URL 설정
const baseURL = import.meta.env.VITE_BASE_URL || 'https://j12e102.p.ssafy.io/api';

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
    if (error.response.status === 401 && !originalRequest._retry) {
      originalRequest._retry = true;
      const { refreshAccessToken, clearToken } = useAuthStore.getState();
      
      try {
        const refreshed = await refreshAccessToken();
        if (refreshed) {
          const { token } = useAuthStore.getState();
          originalRequest.headers.Authorization = `Bearer ${token}`;
          return api(originalRequest);
        } else {
          clearToken();
          // 로그인 페이지로 리다이렉트 또는 다른 처리
          window.location.href = '/login';
          return Promise.reject(error);
        }
      } catch (refreshError) {
        clearToken();
        // 로그인 페이지로 리다이렉트 또는 다른 처리
        window.location.href = '/login';
        return Promise.reject(refreshError);
      }
    }
    return Promise.reject(error);
  }
);

export { api, publicApi };