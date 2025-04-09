import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../hooks/useAuth';
import RegisterForm from '../components/RegisterForm';
import '../styles/AuthForm.css';
import bgImage from '../assets/bgimage.png'; // 배경 이미지 임포트

function RegisterPage() {
  const { register, checkUsername, loading, error } = useAuth();
  const navigate = useNavigate();
  const [showForm, setShowForm] = useState(true);

  const handleSubmit = async (username, email, password, companyId) => {
    try {
      const success = await register(username, email, password, companyId);
      
      if (success) {
        navigate('/login');
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
      {showForm && (
        <RegisterForm
          onSubmit={handleSubmit}
          onCheckUsername={checkUsername}
          loading={loading}
          error={error}
          onBackToLogin={() => {
            setShowForm(false);
            setTimeout(() => navigate('/login'), 300);
          }}
        />
      )}
    </div>
  );
}

export default RegisterPage;
