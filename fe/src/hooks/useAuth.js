import { useState, useCallback } from 'react';
import { api, publicApi } from '../utils/api';
import { jwtDecode } from 'jwt-decode';
import { useAuthStore } from '../stores/authStore';
import { useNavigate } from 'react-router-dom';
import { useToastStore } from '../stores/toastStore';

const useAuth = () => {
  const { 
    setToken, 
    clearToken, 
    refreshAccessToken: storeRefreshToken,
    setIsRefreshing,
    isRefreshing 
  } = useAuthStore();
  const { addToast } = useToastStore();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const navigate = useNavigate();

  // 로그인 함수
  const login = useCallback(async (username, password) => {
    setLoading(true);
    setError(null);
    
    try {
      const response = await publicApi.post('/auth/login', { username, password }, {
        withCredentials: true // 쿠키 포함 설정
      });
      
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
      const errorMessage = error.response?.data?.message || '로그인 중 오류가 발생했습니다.';
      setError(errorMessage);
      
      // 에러 알림
      addToast(errorMessage, 'error');
      
      setLoading(false);
      return false;
    }
  }, [setToken, addToast]);

  // 회원가입 함수
  const register = useCallback(async (username, email, password, companyId) => {
    setLoading(true);
    setError(null);
    
    try {
      const response = await publicApi.post('/auth/register', { 
        username, 
        email, 
        password, 
        company_id: companyId 
      });
      
      console.log('회원가입 성공:', response.data);
      
      // 성공 알림
      addToast('회원가입에 성공했습니다. 로그인해주세요.', 'success');
      
      setLoading(false);
      return true;
    } catch (error) {
      console.error('회원가입 오류:', error);
      const errorMessage = error.response?.data?.message || '회원가입 중 오류가 발생했습니다.';
      setError(errorMessage);
      
      // 에러 알림
      addToast(errorMessage, 'error');
      
      setLoading(false);
      return false;
    }
  }, [addToast]);

  // 아이디 중복 확인 함수
  const checkUsername = useCallback(async (username) => {
    setError(null);
    try {
      // API 엔드포인트 확인 - check-id 또는 check-username
      const response = await publicApi.get('/auth/check-username', {
        params: { username }
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
      const errorMessage = error.response?.data?.message || '사용자명 확인 중 오류가 발생했습니다.';
      setError(errorMessage);
      return false;
    }
  }, []);

  // 토큰 갱신 함수
  const refreshAccessToken = useCallback(async () => {
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
  }, [isRefreshing, setIsRefreshing, storeRefreshToken, addToast]);

  // 로그아웃 함수
  const logout = useCallback(async () => {
    setLoading(true);
    setError(null);
    
    try {
      // 로그아웃 API 호출
      await api.post('/auth/logout', {}, { 
        withCredentials: true // 쿠키 포함 설정
      });
      
      // 토큰 상태 초기화
      clearToken();
      
      // 성공 알림
      addToast('로그아웃되었습니다.', 'success');
      
      setLoading(false);
      return true;
    } catch (error) {
      console.error('로그아웃 오류:', error);
      const errorMessage = error.response?.data?.message || '로그아웃 중 오류가 발생했습니다.';
      setError(errorMessage);
      
      // API 호출 실패해도 클라이언트에서는 로그아웃 처리
      clearToken();
      
      // 에러 알림
      addToast('로그아웃 처리 중 문제가 발생했습니다.', 'warning');
      
      setLoading(false);
      return false;
    }
  }, [clearToken, addToast]);

  // 인증 실패 시 처리 함수
  const handleAuthFailure = useCallback(() => {
    clearToken();
    navigate('/login');
  }, [clearToken, navigate]);

  return { 
    login, 
    register, 
    checkUsername, 
    refreshAccessToken,
    logout, 
    loading, 
    error 
  };
};

export { useAuth };
