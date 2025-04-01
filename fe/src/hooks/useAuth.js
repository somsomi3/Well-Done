import { useState } from 'react';
import { api, publicApi } from '../utils/api';
import { jwtDecode } from 'jwt-decode';
import { useAuthStore } from '../stores/authStore';

const useAuth = () => {
  const { setToken, logout } = useAuthStore();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const login = async (username, password) => {
    setLoading(true);
    setError(null);
    
    try {
      const response = await publicApi.post('/auth/login', { username, password });
      const accessToken = response.data.accessToken;
      setToken(accessToken);
      localStorage.setItem('accessToken', accessToken);
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
      // API 명세에 맞게 요청 형식 수정
      const response = await publicApi.post('/auth/register', { 
        username, 
        email, 
        password, 
        company_id: companyId,
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
      // API 명세에 맞게 엔드포인트 수정
      const response = await publicApi.get('/auth/check-username', {
        params: { username }
      });
      
      console.log('아이디 중복 확인 응답:', response.data);
      // 응답 형식에 맞게 처리
      return response.status === 200;
    } catch (error) {
      console.error('아이디 중복 확인 오류:', error);
      // 400 상태코드는 이미 사용 중인 아이디라는 의미
      if (error.response && error.response.status === 400) {
        setError('이미 사용 중인 사용자 이름입니다.');
        return false;
      }
      setError(error.response?.data?.message || '사용자명 확인 중 오류가 발생했습니다.');
      return false;
    }
  };

  const refreshToken = async () => {
    try {
      const response = await api.get('/auth/refresh-token');
      const newAccessToken = response.data.accessToken;
      setToken(newAccessToken);
      localStorage.setItem('accessToken', newAccessToken);
      return true;
    } catch (error) {
      console.error('토큰 갱신 실패:', error);
      logout();
      return false;
    }
  };

  return { 
    login, 
    register, 
    checkUsername, 
    refreshToken, 
    loading, 
    error 
  };
};

export { useAuth };
