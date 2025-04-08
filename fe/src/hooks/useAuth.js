import { useState, useCallback } from 'react';
import { api, publicApi } from '../utils/api';
import { jwtDecode } from 'jwt-decode';
import { useAuthStore } from '../stores/authStore';
import { useNavigate, useLocation } from 'react-router-dom';
import { useToastStore } from '../stores/toastStore';

// 인증이 필요하지 않은 경로 목록
const PUBLIC_PATHS = ['/login', '/register', '/forgot-password'];

const useAuth = () => {
  const {
    setToken,
    clearToken,
    refreshAccessToken: storeRefreshToken,
    setIsRefreshing,
    isRefreshing,
  } = useAuthStore();
  const { addToast } = useToastStore();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const navigate = useNavigate();
  const location = useLocation();

  // 현재 경로가 인증이 필요하지 않은 경로인지 확인
  const isPublicPath = PUBLIC_PATHS.includes(location.pathname);

  // 인증 실패 시 처리 함수
  const handleAuthFailure = useCallback(() => {
    localStorage.removeItem('accessToken');
    clearToken();
    navigate('/login');
  }, [clearToken, navigate]);

  // 로그인 함수
  const login = useCallback(
    async (username, password) => {
      setLoading(true);
      setError(null);

      try {
        const response = await publicApi.post(
          '/auth/login',
          JSON.stringify({ username, password }),
          {
            withCredentials: true,
            headers: {
              'Content-Type': 'application/json',
            },
          }
        );

        // API 응답 구조 확인
        let accessToken = null;
        if (response.data.accessToken) {
          accessToken = response.data.accessToken;
        } else if (response.data.data) {
          accessToken = response.data.data;
        }

        if (!accessToken) {
          throw new Error('서버 응답에 토큰이 없습니다.');
        }

        // 토큰을 localStorage에 저장
        localStorage.setItem('accessToken', accessToken);

        // 토큰을 스토어에 저장
        setToken(accessToken);

        // 토큰에서 사용자 정보 추출 (디버깅 목적)
        try {
          const decodedToken = jwtDecode(accessToken);
          console.log('로그인 성공, 토큰 정보:', decodedToken);
        } catch (decodeError) {
          console.error('토큰 디코딩 오류:', decodeError);
        }

        // 성공 알림
        addToast('로그인에 성공했습니다.', 'success');

        setLoading(false);
        return true;
      } catch (error) {
        console.error('로그인 오류:', error);
        const errorMessage =
          error.response?.data?.message || '로그인 중 오류가 발생했습니다.';
        setError(errorMessage);

        // 에러 알림
        addToast(errorMessage, 'error');

        setLoading(false);
        return false;
      }
    },
    [setToken, addToast, setLoading, setError]
  );

  // 회원가입 함수
  const register = useCallback(
    async (username, email, password, companyId) => {
      setLoading(true);
      setError(null);

      try {
        const response = await publicApi.post('/auth/register', {
          username,
          email,
          password,
          companyId,
        });

        console.log('회원가입 성공:', response.data);

        // 성공 알림
        addToast('회원가입에 성공했습니다. 로그인해주세요.', 'success');

        setLoading(false);
        return true;
      } catch (error) {
        console.error('회원가입 오류:', error);
        const errorMessage =
          error.response?.data?.message || '회원가입 중 오류가 발생했습니다.';
        setError(errorMessage);

        // 에러 알림
        addToast(errorMessage, 'error');

        setLoading(false);
        return false;
      }
    },
    [addToast, setLoading, setError]
  );

  // 아이디 중복 확인 함수
  const checkUsername = useCallback(
    async (username) => {
      setError(null);
      try {
        // API 엔드포인트 확인 - check-id 또는 check-username
        const response = await publicApi.get('/auth/check-username', {
          params: { username },
        });

        console.log('아이디 중복 확인 응답:', response.data);
        return response.status === 200;
      } catch (error) {
        console.error('아이디 중복 확인 오류:', error);
        if (error.response && error.response.status === 400) {
          const errorMessage = '이미 사용 중인 사용자 이름입니다.';
          setError(errorMessage);
          return false;
        }
        const errorMessage =
          error.response?.data?.message ||
          '사용자명 확인 중 오류가 발생했습니다.';
        setError(errorMessage);
        return false;
      }
    },
    [setError]
  );

  // 토큰 갱신 함수
  const refreshAccessToken = useCallback(async () => {
    // 공개 경로에서는 토큰 갱신을 시도하지 않음
    if (isPublicPath) {
      console.log('공개 페이지에서는 토큰 갱신을 시도하지 않습니다.');
      return false;
    }

    // 이미 리프레시 중이면 중복 요청 방지
    if (isRefreshing) {
      console.log('이미 토큰 리프레시가 진행 중입니다.');
      return false;
    }

    setLoading(true);
    setIsRefreshing(true);

    try {
      // 스토어의 리프레시 토큰 함수 호출
      const success = await storeRefreshToken();
      setLoading(false);
      setIsRefreshing(false);

      if (!success) {
        // 리프레시 실패 시 로그인 페이지로 리다이렉트
        handleAuthFailure();
        addToast('인증이 만료되었습니다. 다시 로그인해주세요.', 'warning');
      }

      return success;
    } catch (error) {
      console.error('토큰 갱신 실패:', error);
      setError('인증이 만료되었습니다. 다시 로그인해주세요.');
      setLoading(false);
      setIsRefreshing(false);

      // 에러 알림
      addToast('인증이 만료되었습니다. 다시 로그인해주세요.', 'error');

      // 에러 발생 시 로그인 페이지로 리다이렉트
      handleAuthFailure();

      return false;
    }
  }, [
    isPublicPath,
    isRefreshing,
    setIsRefreshing,
    storeRefreshToken,
    addToast,
    setLoading,
    setError,
    handleAuthFailure,
  ]);

  // 로그아웃 함수
  const logout = useCallback(async () => {
    setLoading(true);
    setError(null);

    try {
      // 로그아웃 API 호출
      await api.post(
        '/auth/logout',
        {},
        {
          withCredentials: true, // 쿠키 포함 설정
        }
      );

      // localStorage에서 토큰 제거
      localStorage.removeItem('accessToken');

      // 토큰 상태 초기화
      clearToken();

      // 성공 알림
      addToast('로그아웃되었습니다.', 'success');

      setLoading(false);
      return true;
    } catch (error) {
      console.error('로그아웃 오류:', error);
      const errorMessage =
        error.response?.data?.message || '로그아웃 중 오류가 발생했습니다.';
      setError(errorMessage);

      // API 호출 실패해도 클라이언트에서는 로그아웃 처리
      localStorage.removeItem('accessToken');
      clearToken();

      // 에러 알림
      addToast('로그아웃 처리 중 문제가 발생했습니다.', 'warning');

      setLoading(false);
      return false;
    }
  }, [clearToken, addToast, setLoading, setError]);

  return {
    login,
    register,
    checkUsername,
    refreshAccessToken,
    logout,
    loading,
    error,
  };
};

export { useAuth };
