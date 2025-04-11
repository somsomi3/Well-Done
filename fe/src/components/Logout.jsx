import React from 'react';
import { useNavigate } from 'react-router-dom';
import { useAuthStore } from '../stores/authStore';
import { useToastStore } from '../stores/toastStore';
import { api } from '../utils/api';

function Logout() {
  const navigate = useNavigate();
  const { clearToken } = useAuthStore();
  const { addToast } = useToastStore();

  const handleLogout = async () => {
    try {
      // 서버에 로그아웃 요청
      await api.post('/auth/logout');
      
      // 로컬 상태 초기화
      clearToken();
      
      // 사용자에게 알림
      addToast('성공적으로 로그아웃되었습니다.', 'success');
      
      // 로그인 페이지로 리다이렉트
      navigate('/login');
    } catch (error) {
      console.error('로그아웃 중 오류 발생:', error);
      
      // 서버 요청 실패 시에도 로컬 상태는 초기화
      clearToken();
      
      // 사용자에게 오류 알림
      addToast('로그아웃 처리 중 문제가 발생했습니다. 다시 시도해주세요.', 'error');
      
      // 로그인 페이지로 리다이렉트
      navigate('/login');
    }
  };

  return (
    <button
      className="bg-red-500 hover:bg-red-700 text-white font-bold py-2 px-4 rounded focus:outline-none focus:shadow-outline"
      onClick={handleLogout}
    >
      로그아웃
    </button>
  );
}

export default Logout;
