import React from 'react';
import { Navigate } from 'react-router-dom';
import LoginForm from '../components/LoginForm';
import { useAuthStore } from '../stores/authStore';
import '../styles/AuthForm.css';
import bgImage from '../assets/bgimage.png'; // 배경 이미지 임포트

function LoginPage() {
  const { token } = useAuthStore();
  
  // 이미 로그인된 경우 메인 페이지로 리다이렉트
  if (token) {
    return <Navigate to="/main" replace />;
  }
  
  return (
    <div 
      className="auth-container" 
      style={{
        backgroundImage: `url(${bgImage})`,
        backgroundSize: 'cover',
        backgroundPosition: 'center',
        backgroundRepeat: 'no-repeat'
      }}
    >
      <LoginForm />
    </div>
  );
}

export default LoginPage;
