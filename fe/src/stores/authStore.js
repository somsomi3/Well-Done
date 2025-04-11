import { create } from 'zustand';
import { jwtDecode } from 'jwt-decode';
import { api } from '../utils/api';
import { errorService, ErrorCategory } from '../services/errorService';

// 토큰 갱신 최대 재시도 횟수
const MAX_REFRESH_ATTEMPTS = 3;
// 토큰 갱신 요청 간 최소 간격 (밀리초)
const MIN_REFRESH_INTERVAL = 5000;
// 로컬 스토리지 키
const TOKEN_STORAGE_KEY = 'auth_token';
// 마지막 갱신 실패 시간 키
const LAST_REFRESH_FAIL_KEY = 'last_refresh_fail';
// 갱신 실패 후 쿨다운 시간 (밀리초)
const REFRESH_COOLDOWN = 60000; // 1분

const useAuthStore = create((set, get) => ({
  token: null,
  user: null,
  isRefreshing: false,
  refreshAttempts: 0,
  lastRefreshTime: 0,
  
  // 토큰 설정 및 디코딩
  setToken: (token) => {
    if (!token) {
      set({ token: null, user: null, refreshAttempts: 0 });
      localStorage.removeItem(TOKEN_STORAGE_KEY);
      return;
    }
    
    try {
      // 토큰 디코딩하여 사용자 정보 추출
      const decodedToken = jwtDecode(token);
      set({ token, user: decodedToken, refreshAttempts: 0 });
      
      // 로컬 스토리지에 토큰 저장
      localStorage.setItem(TOKEN_STORAGE_KEY, token);
      
      console.log('토큰 설정 완료, 사용자 정보:', decodedToken);
    } catch (error) {
      errorService.logError(error, { action: 'setToken' }, 'warning', ErrorCategory.AUTH);
      set({ token: null, user: null, refreshAttempts: 0 });
      localStorage.removeItem(TOKEN_STORAGE_KEY);
    }
  },
  
  // 토큰 초기화
  clearToken: () => {
    set({ 
      token: null, 
      user: null, 
      isRefreshing: false, 
      refreshAttempts: 0,
      lastRefreshTime: 0
    });
    
    // 로컬 스토리지에서 토큰 제거
    localStorage.removeItem(TOKEN_STORAGE_KEY);
    
    console.log('토큰 상태 초기화 완료');
  },
  
  // 리프레시 상태 설정
  setIsRefreshing: (value) => {
    set({ isRefreshing: value });
  },
  
  // 토큰 유효성 검사
  isTokenValid: () => {
    const { token } = get();
    if (!token) return false;
    
    try {
      const decodedToken = jwtDecode(token);
      const currentTime = Date.now() / 1000;
      
      // 토큰 만료 시간 확인 (5초 여유 추가)
      return decodedToken.exp > currentTime + 5;
    } catch (error) {
      errorService.logError(error, { action: 'isTokenValid' }, 'warning', ErrorCategory.AUTH);
      return false;
    }
  },
  
  // 토큰 만료까지 남은 시간 계산 (초 단위)
  getTokenExpiryTime: () => {
    const { token } = get();
    if (!token) return 0;
    
    try {
      const decodedToken = jwtDecode(token);
      const currentTime = Date.now() / 1000;
      return Math.max(0, decodedToken.exp - currentTime);
    } catch (error) {
      errorService.logError(error, { action: 'getTokenExpiryTime' }, 'warning', ErrorCategory.AUTH);
      return 0;
    }
  },
  
  // 액세스 토큰 리프레시
  refreshAccessToken: async () => {
    // 이미 리프레시 중이면 중복 요청 방지
    if (get().isRefreshing) {
      console.log('이미 토큰 리프레시가 진행 중입니다.');
      return false;
    }
    
    // 최대 재시도 횟수 초과 확인
    if (get().refreshAttempts >= MAX_REFRESH_ATTEMPTS) {
      console.error(`토큰 리프레시 최대 재시도 횟수(${MAX_REFRESH_ATTEMPTS}회) 초과`);
      get().clearToken();
      return false;
    }
    
    // 마지막 갱신 실패 후 쿨다운 시간 확인
    const lastFailTime = parseInt(localStorage.getItem(LAST_REFRESH_FAIL_KEY) || '0', 10);
    const now = Date.now();
    const timeSinceLastFail = now - lastFailTime;
    
    if (lastFailTime > 0 && timeSinceLastFail < REFRESH_COOLDOWN) {
      const remainingCooldown = Math.ceil((REFRESH_COOLDOWN - timeSinceLastFail) / 1000);
      console.log(`토큰 갱신 쿨다운 중입니다. ${remainingCooldown}초 후에 다시 시도하세요.`);
      return false;
    }
    
    // 마지막 리프레시 시도 후 최소 간격 확인
    const timeSinceLastRefresh = now - get().lastRefreshTime;
    if (timeSinceLastRefresh < MIN_REFRESH_INTERVAL) {
      console.log(`토큰 리프레시 요청이 너무 빈번합니다. ${Math.ceil((MIN_REFRESH_INTERVAL - timeSinceLastRefresh) / 1000)}초 후 다시 시도하세요.`);
      return false;
    }
    
    set({ isRefreshing: true, lastRefreshTime: now });
    set(state => ({ refreshAttempts: state.refreshAttempts + 1 }));
    
    try {
      console.log('토큰 리프레시 시도...', get().refreshAttempts);
      
      const response = await api.post('/auth/refresh', {}, {
        withCredentials: true // 쿠키 포함 설정
      });
      
      // API 응답 구조 확인
      const responseData = response.data;
      console.log('리프레시 응답:', responseData);
      
      // 응답 구조에 따라 토큰 추출
      let newToken = null;
      if (responseData.accessToken) {
        newToken = responseData.accessToken;
      } else if (responseData.data) {
        newToken = responseData.data;
      } else if (typeof responseData === 'string') {
        newToken = responseData;
      }
      
      if (!newToken) {
        throw new Error('토큰 리프레시 응답에서 토큰을 찾을 수 없습니다.');
      }
      
      // 새 토큰 설정
      get().setToken(newToken);
      console.log('토큰 리프레시 성공');
      
      // 성공 시 실패 기록 초기화
      localStorage.removeItem(LAST_REFRESH_FAIL_KEY);
      set({ isRefreshing: false, refreshAttempts: 0 });
      return true;
    } catch (error) {
      const errorInfo = {
        action: 'refreshAccessToken',
        attempt: get().refreshAttempts,
        maxAttempts: MAX_REFRESH_ATTEMPTS,
        status: error.response?.status,
        message: error.response?.data?.message || error.message
      };
      
      errorService.logError(error, errorInfo, 'error', ErrorCategory.AUTH);
      console.error('액세스 토큰 리프레시 오류:', error.message);
      
      // 실패 시간 기록
      localStorage.setItem(LAST_REFRESH_FAIL_KEY, Date.now().toString());
      
      // 401 오류가 계속 발생하면 로그아웃 처리
      if (error.response?.status === 401 && get().refreshAttempts >= 2) {
        console.log('리프레시 토큰이 유효하지 않습니다. 로그아웃 처리합니다.');
        get().clearToken();
        set({ isRefreshing: false });
        return false;
      }
      
      // 일시적인 오류일 수 있으므로 리프레싱 상태만 해제
      set({ isRefreshing: false });
      return false;
    }
  },
  
  // 사용자 인증 상태 확인
  isAuthenticated: () => {
    return get().isTokenValid();
  },
  
  // 사용자 정보 가져오기
  getUserInfo: () => {
    return get().user;
  },
  
  // 초기화 함수 - 앱 시작 시 로컬 스토리지에서 토큰 복원
  initializeAuth: () => {
    try {
      const storedToken = localStorage.getItem(TOKEN_STORAGE_KEY);
      if (storedToken) {
        get().setToken(storedToken);
        console.log('로컬 스토리지에서 토큰 복원 완료');
        return true;
      }
    } catch (error) {
      errorService.logError(error, { action: 'initializeAuth' }, 'warning', ErrorCategory.AUTH);
      localStorage.removeItem(TOKEN_STORAGE_KEY);
    }
    return false;
  },
  
  // 디버그 정보 출력
  debugAuthState: () => {
    const state = get();
    console.log('===== 인증 상태 디버그 =====');
    console.log('토큰 존재:', !!state.token);
    console.log('사용자 정보:', state.user);
    console.log('토큰 유효성:', state.isTokenValid());
    console.log('만료까지 남은 시간:', state.getTokenExpiryTime(), '초');
    console.log('리프레시 중:', state.isRefreshing);
    console.log('리프레시 시도 횟수:', state.refreshAttempts);
    console.log('마지막 리프레시 시간:', new Date(state.lastRefreshTime).toLocaleString());
    console.log('===========================');
  }
}));

export { useAuthStore };
