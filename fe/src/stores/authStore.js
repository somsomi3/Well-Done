import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import { jwtDecode } from 'jwt-decode';
import { api } from '../utils/api';

const useAuthStore = create(
  persist(
    (set, get) => ({
      token: null,
      user: null,
      
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
        set({ token: null, user: null });
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
      
      // 액세스 토큰 리프레시
      refreshAccessToken: async () => {
        try {
          const response = await api.post('/auth/refresh');
          const newToken = response.data.accessToken;
          
          // 새 토큰 설정
          get().setToken(newToken);
          return true;
        } catch (error) {
          console.error('액세스 토큰 리프레시 오류:', error);
          // 리프레시 실패 시 로그아웃 처리
          get().clearToken();
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
    }),
    {
      name: 'auth-storage',
      storage: createJSONStorage(() => sessionStorage), // 세션 스토리지 사용
    }
  )
);

export { useAuthStore };
