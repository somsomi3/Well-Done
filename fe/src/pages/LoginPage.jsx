import React from 'react';
import { Navigate } from 'react-router-dom';
import LoginForm from '../components/LoginForm';
import { useAuthStore } from '../stores/authStore';

function LoginPage() {
  const { token } = useAuthStore();
  
  // 이미 로그인된 경우 메인 페이지로 리다이렉트
  if (token) {
    return <Navigate to="/main" replace />;
  }
  
  return (
    <div className="min-h-screen flex items-center justify-center bg-gray-100 px-4">
      <div className="max-w-md w-full">
        <LoginForm />
      </div>
    </div>
  );
}

export default LoginPage;
