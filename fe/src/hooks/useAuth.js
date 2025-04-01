import { useState } from 'react';
import { api, publicApi } from '../../../utils/api';
import { jwtDecode } from 'jwt-decode';
import { useAuthStore } from '../store/authStore';
import { useNavigate } from 'react-router-dom';

const useAuth = () => {
  const { setToken, clearToken, refreshAccessToken: storeRefreshToken } = useAuthStore();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const navigate = useNavigate();

  const login = async (username, password) => {
    setLoading(true);
    setError(null);
    
    try {
      const response = await publicApi.post('/auth/login', { username, password });
      const accessToken = response.data.accessToken;
      
      // 토큰을 스토어에만 저장
      setToken(accessToken);
      
      // 토큰에서 사용자 정보 추출 (디버깅 목적)
      try {
        const decodedToken = jwtDecode(accessToken);
        console.log('로그인 성공, 토큰 정보:', decodedToken);
      } catch (decodeError) {
        console.error('토큰 디코딩 오류:', decodeError);
      }
      
      setLoading(false);
      return true;
    } catch (error) {
      console.error('로그인 오류:', error);
      setError(error.response?.data?.message || '로그인 중 오류가 발생했습니다.');
      setLoading(false);
      return false;
    }
  };

  const register = async (username, email, password, companyId) => {
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
      setLoading(false);
      return true;
    } catch (error) {
      console.error('회원가입 오류:', error);
      setError(error.response?.data?.message || '회원가입 중 오류가 발생했습니다.');
      setLoading(false);
      return false;
    }
  };

  const checkUsername = async (username) => {
    setError(null);
    try {
      const response = await publicApi.get('/auth/check-id', {
        params: { username }
      });
      
      console.log('아이디 중복 확인 응답:', response.data);
      return response.status === 200;
    } catch (error) {
      console.error('아이디 중복 확인 오류:', error);
      if (error.response && error.response.status === 400) {
        setError('이미 사용 중인 사용자 이름입니다.');
        return false;
      }
      setError(error.response?.data?.message || '사용자명 확인 중 오류가 발생했습니다.');
      return false;
    }
  };

  const refreshAccessToken = async () => {
    setLoading(true);
    try {
      // 스토어의 리프레시 토큰 함수 호출
      const success = await storeRefreshToken();
      setLoading(false);
      
      if (!success) {
        // 리프레시 실패 시 로그인 페이지로 리다이렉트
        handleAuthFailure();
      }
      
      return success;
    } catch (error) {
      console.error('토큰 갱신 실패:', error);
      setError('인증이 만료되었습니다. 다시 로그인해주세요.');
      setLoading(false);
      
      // 에러 발생 시 로그인 페이지로 리다이렉트
      handleAuthFailure();
      
      return false;
    }
  };

  const logout = async () => {
    setLoading(true);
    setError(null);
    
    try {
      // 로그아웃 API 호출
      await api.post('/auth/logout');
      
      // 토큰 상태 초기화
      clearToken();
      
      setLoading(false);
      return true;
    } catch (error) {
      console.error('로그아웃 오류:', error);
      setError(error.response?.data?.message || '로그아웃 중 오류가 발생했습니다.');
      
      // API 호출 실패해도 클라이언트에서는 로그아웃 처리
      clearToken();
      
      setLoading(false);
      return false;
    }
  };

  // 인증 실패 시 처리 함수
  const handleAuthFailure = () => {
    clearToken();
    navigate('/login');
  };

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
