import React, { useState } from 'react';
import AlertModal from './AlertModal';
import '../styles/AuthForm.css';
import logo from '../assets/logo.png';

function RegisterForm({ 
  onSubmit, 
  onCheckUsername, 
  loading, 
  error 
}) {
  const [username, setUsername] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [companyId, setCompanyId] = useState('');
  const [isUsernameAvailable, setIsUsernameAvailable] = useState(null);
  const [isCheckingUsername, setIsCheckingUsername] = useState(false);
  const [usernameChecked, setUsernameChecked] = useState(false);
  
  // 모달 상태
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [modalTitle, setModalTitle] = useState('');
  const [modalMessage, setModalMessage] = useState('');

  const handleCheckUsername = async () => {
    if (!username || username.length < 3) {
      showModal('입력 오류', '아이디는 최소 3자 이상이어야 합니다.');
      return;
    }
    
    setIsCheckingUsername(true);
    const available = await onCheckUsername(username);
    setIsUsernameAvailable(available);
    setUsernameChecked(true);
    setIsCheckingUsername(false);
    
    if (available) {
      showModal('사용 가능', '사용 가능한 아이디입니다.');
    } else {
      showModal('사용 불가', '이미 사용 중인 아이디입니다.');
    }
  };

  const showModal = (title, message) => {
    setModalTitle(title);
    setModalMessage(message);
    setIsModalOpen(true);
  };

  const handleUsernameChange = (e) => {
    setUsername(e.target.value);
    setIsUsernameAvailable(null);
    setUsernameChecked(false);
  };

  const handleSubmit = (event) => {
    event.preventDefault();
    if (!usernameChecked || !isUsernameAvailable) {
      showModal('확인 필요', '아이디 중복 확인을 먼저 해주세요.');
      return;
    }
    onSubmit(username, email, password, companyId);
  };

  return (
    <>
      <form onSubmit={handleSubmit} className="auth-form">
        <div className="flex flex-col items-center mb-6">
          <img src={logo} alt="Well-Done Logo" className="h-16 mb-2" />
          <h1 className="text-3xl font-bold app-title">Well-Done</h1>
          <h2 className="text-xl text-gray-600">회원가입</h2>
        </div>
        
        {error && <div className="mb-4 p-2 bg-red-100 text-red-700 rounded">{error}</div>}
        
        {/* 아이디 입력 폼 그룹 */}
        <div className="form-group">
          <label className="block text-gray-700 font-bold mb-2" htmlFor="username">
            아이디
          </label>
          <div className="flex">
            <input
              className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
              type="text"
              name="username"
              value={username}
              onChange={handleUsernameChange}
              placeholder="아이디"
              required
              minLength={3}
            />
            <button
              type="button"
              className="ml-2 btn-small bg-gray-300 hover:bg-gray-400 text-gray-800 font-bold rounded focus:outline-none focus:shadow-outline username-check-button"
              onClick={handleCheckUsername}
              disabled={isCheckingUsername || !username || username.length < 3}
            >
              {isCheckingUsername ? '확인 중...' : '중복 확인'}
            </button>
          </div>
        </div>
        
        {/* 이메일 입력 폼 그룹 */}
        <div className="form-group">
          <label className="block text-gray-700 font-bold mb-2" htmlFor="email">
            이메일
          </label>
          <input
            className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
            type="email"
            name="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="이메일"
            required
          />
        </div>
        
        {/* 비밀번호 입력 폼 그룹 */}
        <div className="form-group">
          <label className="block text-gray-700 font-bold mb-2" htmlFor="password">
            비밀번호
          </label>
          <input
            className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
            type="password"
            name="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            placeholder="비밀번호"
            required
            minLength={4}
          />
        </div>
        
        {/* 회사 코드 입력 폼 그룹 */}
        <div className="form-group">
          <label className="block text-gray-700 font-bold mb-2" htmlFor="companyId">
            회사 코드
          </label>
          <input
            className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
            type="text"
            name="companyId"
            value={companyId}
            onChange={(e) => setCompanyId(e.target.value)}
            placeholder="회사 코드"
            required
          />
        </div>
        
        {/* 버튼 그룹 */}
        <div className="button-group">
          <button
            className={`btn-large bg-blue-500 hover:bg-blue-700 text-white font-bold rounded focus:outline-none focus:shadow-outline ${loading || !usernameChecked || !isUsernameAvailable ? 'opacity-50 cursor-not-allowed' : ''}`}
            type="submit"
            disabled={loading || !usernameChecked || !isUsernameAvailable}
          >
            {loading ? '처리 중...' : '회원가입'}
          </button>
          <button
            className="btn-small bg-gray-300 hover:bg-gray-400 text-gray-800 font-bold rounded focus:outline-none focus:shadow-outline"
            type="button"
            onClick={() => window.location.href = '/login'}
          >
            로그인 페이지로
          </button>
        </div>
      </form>

      <AlertModal
        isOpen={isModalOpen}
        onClose={() => setIsModalOpen(false)}
        title={modalTitle}
        message={modalMessage}
      />
    </>
  );
}

export default RegisterForm;
