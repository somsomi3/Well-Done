import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../hooks/useAuth';
import AlertModal from './AlertModal';
import '../styles/AuthForm.css';
import logo from '../assets/logo.png';

function LoginForm() {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [modalMessage, setModalMessage] = useState('');
  const { login, loading, error } = useAuth();
  const navigate = useNavigate();

  const handleSubmit = async (e) => {
    e.preventDefault();
    
    if (!username || !password) {
      setModalMessage('아이디와 비밀번호를 모두 입력해주세요.');
      setIsModalOpen(true);
      return;
    }
    
    const success = await login(username, password);
    
    if (success) {
      navigate('/main');
    } else if (error) {
      setModalMessage(error);
      setIsModalOpen(true);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="auth-form">
      <div className="flex flex-col items-center mb-6">
        <img src={logo} alt="Well-Done Logo" className="h-16 mb-2" />
        <h1 className="text-3xl font-bold app-title">Well-Done</h1>
        <h2 className="text-xl text-gray-600">로그인</h2>
      </div>
      
      <div className="mb-4">
        <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="username">
          아이디
        </label>
        <input
          className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
          id="username"
          type="text"
          placeholder="아이디"
          value={username}
          onChange={(e) => setUsername(e.target.value)}
        />
      </div>
      
      <div className="mb-6">
        <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="password">
          비밀번호
        </label>
        <input
          className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 mb-3 leading-tight focus:outline-none focus:shadow-outline"
          id="password"
          type="password"
          placeholder="비밀번호"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
        />
      </div>
      
      <div className="flex items-center justify-between">
        <button
          className={`bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded focus:outline-none focus:shadow-outline ${loading ? 'opacity-50 cursor-not-allowed' : ''}`}
          type="submit"
          disabled={loading}
        >
          {loading ? '로그인 중...' : '로그인'}
        </button>
        <button
          className="inline-block align-baseline font-bold text-sm text-blue-500 hover:text-blue-800"
          type="button"
          onClick={() => navigate('/register')}
        >
          회원가입
        </button>
      </div>
      
      <AlertModal
        isOpen={isModalOpen}
        onClose={() => setIsModalOpen(false)}
        title="알림"
        message={modalMessage}
      />
    </form>
  );
}

export default LoginForm;
