import { create } from 'zustand';
import { jwtDecode } from 'jwt-decode';
import { api } from '../utils/api';

const useAuthStore = create((set, get) => ({
  token: null,
  user: null,
  isRefreshing: false, // 리프레시 중인지 추적하는 플래그를 스토어 내부로 이동
  
  // 토큰 설정 및 디코딩
  setToken: (token) => {
    if (!token) {
      set({ token: null, user: null });
      return;
    }
    
    try {
      // 토큰 디코딩하여 사용자 정보 추출
      const decodedToken = jwtDecode(token);
      set({ token, user: decodedToken });
    } catch (error) {
      console.error('토큰 디코딩 오류:', error);
      set({ token: null, user: null });
    }
  },
  
  // 토큰 초기화
  clearToken: () => {
    set({ token: null, user: null, isRefreshing: false });
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
      
      // 토큰 만료 시간 확인
      return decodedToken.exp > currentTime;
    } catch (error) {
      console.error('토큰 유효성 검사 오류:', error);
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
      console.error('토큰 만료 시간 계산 오류:', error);
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
    
    set({ isRefreshing: true });
    
    try {
      console.log('토큰 리프레시 시도...');
      const response = await api.post('/auth/refresh');
      const newToken = response.data.accessToken;
      
      // 새 토큰 설정
      get().setToken(newToken);
      console.log('토큰 리프레시 성공');
      
      set({ isRefreshing: false });
      return true;
    } catch (error) {
      console.error('액세스 토큰 리프레시 오류:', error);
      // 리프레시 실패 시 로그아웃 처리
      get().clearToken();
      console.log('토큰 리프레시 실패, 로그아웃 처리됨');
      
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
  }
}));

export { useAuthStore };
