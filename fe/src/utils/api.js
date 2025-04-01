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
    console.log('API 요청 설정:', {
      url: `${config.baseURL}${config.url}`,
      method: config.method,
      headers: config.headers
    });
    
    // 스토어에서 토큰 가져오기
    const { token } = useAuthStore.getState();
    
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  (error) => {
    console.error('API 요청 오류:', error);
    return Promise.reject(error);
  }
);

// 응답 인터셉터 - 토큰 만료 시 리프레시 처리
api.interceptors.response.use(
  (response) => {
    console.log('API 응답 성공:', {
      status: response.status,
      url: `${response.config.baseURL}${response.config.url}`,
      data: response.data
    });
    return response;
  },
  async (error) => {
    const originalRequest = error.config;
    
    // 토큰 만료로 인한 401 오류이고, 아직 재시도하지 않은 경우
    if (error.response && error.response.status === 401 && !originalRequest._retry) {
      originalRequest._retry = true;
      
      try {
        // 토큰 리프레시 시도
        const authStore = useAuthStore.getState();
        const refreshSuccess = await authStore.refreshAccessToken();
        
        if (refreshSuccess) {
          // 토큰 리프레시 성공 시 원래 요청 재시도
          const newToken = authStore.token;
          originalRequest.headers.Authorization = `Bearer ${newToken}`;
          return api(originalRequest);
        } else {
          // 리프레시 실패 시 로그인 페이지로 리다이렉트
          window.location.href = '/login';
          return Promise.reject(error);
        }
      } catch (refreshError) {
        console.error('토큰 리프레시 오류:', refreshError);
        // 리프레시 실패 시 로그인 페이지로 리다이렉트
        window.location.href = '/login';
        return Promise.reject(refreshError);
      }
    }
    
    // 기타 오류 처리
    if (error.response) {
      console.error('API 응답 오류:', {
        status: error.response.status,
        statusText: error.response.statusText,
        url: `${error.config.baseURL}${error.config.url}`,
        data: error.response.data
      });
    } else if (error.request) {
      console.error('API 응답을 받지 못했습니다:', {
        request: error.request,
        message: '서버에 연결할 수 없거나 응답이 없습니다.'
      });
    } else {
      console.error('API 요청 설정 중 오류:', error.message);
    }
    
    return Promise.reject(error);
  }
);

// 인증이 필요없는 요청의 인터셉터 설정
publicApi.interceptors.request.use(
  (config) => {
    console.log('Public API 요청 설정:', {
      url: `${config.baseURL}${config.url}`,
      method: config.method,
      data: config.data
    });
    return config;
  },
  (error) => {
    console.error('Public API 요청 오류:', error);
    return Promise.reject(error);
  }
);

publicApi.interceptors.response.use(
  (response) => {
    console.log('Public API 응답 성공:', {
      status: response.status,
      url: `${response.config.baseURL}${response.config.url}`,
      data: response.data
    });
    return response;
  },
  (error) => {
    if (error.response) {
      console.error('Public API 응답 오류:', {
        status: error.response.status,
        statusText: error.response.statusText,
        url: `${error.config.baseURL}${error.config.url}`,
        data: error.response.data
      });
    } else if (error.request) {
      console.error('Public API 응답을 받지 못했습니다:', {
        message: '서버에 연결할 수 없거나 응답이 없습니다.'
      });
    } else {
      console.error('Public API 요청 설정 중 오류:', error.message);
    }
    
    return Promise.reject(error);
  }
);

export { api, publicApi };