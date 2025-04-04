import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../hooks/useAuth';
import RegisterForm from '../components/RegisterForm';
import '../styles/AuthForm.css';
import bgImage from '../assets/bgimage.png'; // 배경 이미지 임포트

function RegisterPage() {
  const { register, checkUsername, loading, error } = useAuth();
  const navigate = useNavigate();
  const [registrationSuccess, setRegistrationSuccess] = useState(false);

  const handleCheckUsername = async (username) => {
    try {
      const available = await checkUsername(username);
      return available;
    } catch (err) {
      console.error('아이디 중복 확인 오류:', err);
      return false;
    }
  };

  const handleSubmit = async (username, email, password, companyId) => {
    try {
      const success = await register(username, email, password, companyId);
      
      if (success) {
        setRegistrationSuccess(true);
        setTimeout(() => {
          navigate('/login');
        }, 3000);
      }
    } catch (err) {
      console.error('회원가입 오류:', err);
    }
  };

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
      {registrationSuccess ? (
        <div className="auth-form bg-green-100 border border-green-400 text-green-700 px-4 py-3 rounded relative mb-4">
          <div className="flex flex-col items-center mb-6">
            <h1 className="text-3xl font-bold app-title">Well-Done</h1>
            <h2 className="text-xl text-green-700">회원가입 성공!</h2>
          </div>
          <p className="text-center mb-4">잠시 후 로그인 페이지로 이동합니다.</p>
        </div>
      ) : (
        <RegisterForm
          onSubmit={handleSubmit}
          onCheckUsername={handleCheckUsername}
          loading={loading}
          error={error}
        />
      )}
    </div>
  );
}

export default RegisterPage;
