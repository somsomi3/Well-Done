import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../hooks/useAuth';
import RegisterForm from '../components/RegisterForm';

function Register() {
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
    <div className="flex justify-center items-center min-h-screen p-4">
      <div className="w-full max-w-md">
        {registrationSuccess ? (
          <div className="bg-green-100 border border-green-400 text-green-700 px-4 py-3 rounded relative mb-4">
            <strong className="font-bold">회원가입 성공!</strong>
            <p className="block sm:inline">잠시 후 로그인 페이지로 이동합니다.</p>
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
    </div>
  );
}

export default Register;
