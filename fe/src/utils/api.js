import axios from 'axios';
import { useAuthStore } from '../stores/authStore';
import { getApiUrl } from '../configs/env';
import { errorService, ErrorCategory } from '../services/errorService';

// API 기본 URL 설정 - 환경 변수에서 가져옴
const baseURL = getApiUrl();

// 인증이 필요한 요청을 위한 인스턴스
const api = axios.create({
  baseURL,
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  },
  withCredentials: true // 쿠키 포함 설정
});

// 인증이 필요없는 요청을 위한 인스턴스
const publicApi = axios.create({
  baseURL,
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  },
});

// 진행 중인 토큰 갱신 요청을 추적하기 위한 변수
let refreshTokenPromise = null;
// 토큰 갱신 요청 실패 횟수
let refreshFailCount = 0;
// 최대 실패 허용 횟수
const MAX_REFRESH_FAILS = 3;

// 요청 인터셉터 - 인증이 필요한 요청에 토큰 추가
api.interceptors.request.use(
  (config) => {
    const { token } = useAuthStore.getState();

    // ✅ /auth 요청은 토큰 붙이지 않음
    const isAuthUrl =
      config.url?.includes('/auth/login') ||
      config.url?.includes('/auth/register') ||
      config.url?.includes('/auth/refresh');

    if (token && !isAuthUrl) {
      config.headers.Authorization = `Bearer ${token}`;
    }

    if (config.withCredentials === undefined) {
      config.withCredentials = true;
    }

    return config;
  },
  (error) => {
    errorService.handleNetworkError(error, { type: 'request' });
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
      // 최대 실패 횟수 초과 확인
      if (refreshFailCount >= MAX_REFRESH_FAILS) {
        console.error(`토큰 갱신 최대 실패 횟수(${MAX_REFRESH_FAILS}회) 초과`);
        const { clearToken } = useAuthStore.getState();
        clearToken();
        
        // 로그인 페이지로 리다이렉트
        if (typeof window !== 'undefined') {
          window.location.href = '/login';
        }
        
        return Promise.reject(error);
      }
      
      originalRequest._retry = true;
      
      try {
        // 이미 진행 중인 토큰 갱신 요청이 있으면 그 결과를 기다림
        if (refreshTokenPromise) {
          const result = await refreshTokenPromise;
          if (!result) {
            throw new Error('토큰 갱신 실패');
          }
          
          // 토큰이 갱신되었는지 확인
          const { token } = useAuthStore.getState();
          if (token) {
            // 토큰이 갱신되었으면 원래 요청 재시도
            originalRequest.headers.Authorization = `Bearer ${token}`;
            return api(originalRequest);
          }
          
          // 토큰 갱신 실패 시 에러 반환
          throw new Error('토큰 갱신 후에도 토큰이 없음');
        }
        
        // 토큰 갱신 요청 시작
        const { refreshAccessToken, clearToken } = useAuthStore.getState();
        
        // 토큰 갱신 요청을 Promise로 추적
        refreshTokenPromise = refreshAccessToken();
        
        try {
          // 토큰 갱신 요청 완료 대기
          const refreshed = await refreshTokenPromise;
          
          if (refreshed) {
            // 토큰이 갱신되었으면 원래 요청 재시도
            const { token } = useAuthStore.getState();
            originalRequest.headers.Authorization = `Bearer ${token}`;
            refreshFailCount = 0; // 성공 시 실패 카운트 초기화
            return api(originalRequest);
          } else {
            // 토큰 갱신 실패 시 실패 카운트 증가
            refreshFailCount++;
            
            // 실패 횟수가 임계값을 넘으면 로그아웃 처리
            if (refreshFailCount >= MAX_REFRESH_FAILS) {
              clearToken();
              
              // 로그인 페이지로 리다이렉트
              if (typeof window !== 'undefined') {
                window.location.href = '/login';
              }
            }
            
            throw new Error('토큰 갱신 실패');
          }
        } finally {
          // 토큰 갱신 요청 추적 변수 초기화
          refreshTokenPromise = null;
        }
      } catch (refreshError) {
        // 토큰 갱신 과정에서 오류 발생 시
        errorService.handleAuthError(refreshError);
        refreshFailCount++;
        
        // 로그아웃 처리
        if (refreshFailCount >= MAX_REFRESH_FAILS) {
          const { clearToken } = useAuthStore.getState();
          clearToken();
          
          // 로그인 페이지로 리다이렉트
          if (typeof window !== 'undefined') {
            window.location.href = '/login';
          }
        }
        
        return Promise.reject(refreshError);
      }
    }
    
    // 네트워크 오류 처리
    if (!error.response) {
      errorService.handleNetworkError(error, { url: originalRequest?.url });
    }
    // API 오류 처리
    else {
      const apiInfo = {
        url: originalRequest?.url,
        method: originalRequest?.method,
        status: error.response.status,
        statusText: error.response.statusText
      };
      errorService.handleApiError(error, apiInfo);
    }
    
    // 다른 종류의 에러는 그대로 반환
    return Promise.reject(error);
  }
);

// 공개 API 요청에 대한 인터셉터 설정
publicApi.interceptors.request.use(
  (config) => {
    // 로그인, 회원가입 등 인증 관련 요청에는 withCredentials 옵션 추가
    if (config.url && (
      config.url.includes('/auth/login') || 
      config.url.includes('/auth/register') || 
      config.url.includes('/auth/refresh')
    )) {
      config.withCredentials = true;
    }
    return config;
  },
  (error) => {
    errorService.handleNetworkError(error, { type: 'public-request' });
    return Promise.reject(error);
  }
);

publicApi.interceptors.response.use(
  (response) => response,
  (error) => {
    // 네트워크 오류 처리
    if (!error.response) {
      errorService.handleNetworkError(error, { type: 'public-response' });
    }
    // API 오류 처리
    else {
      const apiInfo = {
        url: error.config?.url,
        method: error.config?.method,
        status: error.response.status,
        statusText: error.response.statusText,
        type: 'public'
      };
      errorService.handleApiError(error, apiInfo);
    }
    
    return Promise.reject(error);
  }
);

export { api, publicApi };
