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
      navigate('/announcements');
    } else if (error) {
      setModalMessage(error);
      setIsModalOpen(true);
    }
  };

  // ID 찾기 기능 (추후 구현 예정)
  const handleFindId = () => {
    setModalMessage('ID 찾기 기능은 현재 개발 중입니다.');
    setIsModalOpen(true);
  };

  // 비밀번호 찾기 기능 (추후 구현 예정)
  const handleFindPassword = () => {
    setModalMessage('비밀번호 찾기 기능은 현재 개발 중입니다.');
    setIsModalOpen(true);
  };

  return (
    <form onSubmit={handleSubmit} className="auth-form">
      <div className="flex flex-col items-center mb-6">
        <img src={logo} alt="Well-Done Logo" className="h-16 mb-2" />
        <h1 className="text-3xl font-bold app-title">Well-Done</h1>
        <h2 className="text-xl text-gray-600">로그인</h2>
      </div>

      {/* 아이디 입력 폼 그룹 */}
      <div className="form-group">
        <label
          className="block text-gray-700 font-bold mb-2"
          htmlFor="username"
        >
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

      {/* 비밀번호 입력 폼 그룹 */}
      <div className="form-group">
        <label
          className="block text-gray-700 font-bold mb-2"
          htmlFor="password"
        >
          비밀번호
        </label>
        <input
          className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
          id="password"
          type="password"
          placeholder="비밀번호"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
        />
      </div>

      {/* 로그인 버튼 그룹 */}
      <div className="form-group flex justify-center">
        <button
          className="btn-large bg-blue-500 hover:bg-blue-700 text-white font-bold rounded focus:outline-none focus:shadow-outline"
          type="submit"
          disabled={loading}
        >
          {loading ? '로그인 중...' : '로그인'}
        </button>
      </div>

      {/* ID 찾기, PW 찾기, 회원가입 버튼 그룹 */}
      <div className="form-group flex flex-wrap justify-center gap-2">
        <button
          type="button"
          onClick={handleFindId}
          className="btn-small bg-gray-200 hover:bg-gray-300 text-gray-700 font-bold rounded focus:outline-none focus:shadow-outline"
        >
          ID 찾기
        </button>
        <button
          type="button"
          onClick={handleFindPassword}
          className="btn-small bg-gray-200 hover:bg-gray-300 text-gray-700 font-bold rounded focus:outline-none focus:shadow-outline"
        >
          PW 찾기
        </button>
        <button
          type="button"
          onClick={() => navigate('/register')}
          className="btn-small bg-gray-200 hover:bg-gray-300 text-gray-700 font-bold rounded focus:outline-none focus:shadow-outline"
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
