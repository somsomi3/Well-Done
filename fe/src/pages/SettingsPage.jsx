import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import Layout from '../components/Layout/Layout';
import AlertModal from '../components/AlertModal';
import { useAuthStore } from '../stores/authStore';
import { api } from '../utils/api';

function SettingsPage() {
  const navigate = useNavigate();
  const { logout } = useAuthStore();
  const [activeTab, setActiveTab] = useState('profile');
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [modalTitle, setModalTitle] = useState('');
  const [modalMessage, setModalMessage] = useState('');
  
  // 사용자 프로필 정보 (실제로는 API에서 가져올 것입니다)
  const [userProfile, setUserProfile] = useState({
    username: 'admin',
    email: 'admin@example.com',
    company: '로봇 관리 시스템',
    role: '관리자'
  });
  
  // 시스템 설정 (실제로는 API에서 가져올 것입니다)
  const [systemSettings, setSystemSettings] = useState({
    notificationsEnabled: true,
    darkMode: false,
    autoRefresh: true,
    refreshInterval: 5
  });
  
  const handleProfileSubmit = (e) => {
    e.preventDefault();
    // 실제로는 API 호출을 통해 프로필 정보를 업데이트합니다
    showModal('성공', '프로필 정보가 성공적으로 업데이트되었습니다.');
  };
  
  const handleSystemSettingsSubmit = (e) => {
    e.preventDefault();
    // 실제로는 API 호출을 통해 시스템 설정을 업데이트합니다
    showModal('성공', '시스템 설정이 성공적으로 업데이트되었습니다.');
  };
  
  const handlePasswordSubmit = (e) => {
    e.preventDefault();
    // 실제로는 API 호출을 통해 비밀번호를 변경합니다
    showModal('성공', '비밀번호가 성공적으로 변경되었습니다.');
  };
  
  const handleLogout = async () => {
    try {
      // 로그아웃 API 호출
      await api.post('/auth/logout');
      // 로컬 상태 초기화
      logout();
      // 로그인 페이지로 이동
      navigate('/login');
    } catch (error) {
      console.error('로그아웃 오류:', error);
      showModal('오류', '로그아웃 중 문제가 발생했습니다.');
    }
  };
  
  const showModal = (title, message) => {
    setModalTitle(title);
    setModalMessage(message);
    setIsModalOpen(true);
  };

  return (
    <Layout>
      <div className="p-6">
        <h1 className="text-2xl font-bold mb-6">설정</h1>
        
        {/* 탭 메뉴 */}
        <div className="flex border-b mb-6">
          <button 
            className={`py-2 px-4 font-medium ${activeTab === 'profile' ? 'border-b-2 border-blue-500 text-blue-500' : 'text-gray-500'}`}
            onClick={() => setActiveTab('profile')}
          >
            프로필 정보
          </button>
          <button 
            className={`py-2 px-4 font-medium ${activeTab === 'password' ? 'border-b-2 border-blue-500 text-blue-500' : 'text-gray-500'}`}
            onClick={() => setActiveTab('password')}
          >
            비밀번호 변경
          </button>
          <button 
            className={`py-2 px-4 font-medium ${activeTab === 'system' ? 'border-b-2 border-blue-500 text-blue-500' : 'text-gray-500'}`}
            onClick={() => setActiveTab('system')}
          >
            시스템 설정
          </button>
        </div>
        
        {/* 프로필 정보 탭 */}
        {activeTab === 'profile' && (
          <form onSubmit={handleProfileSubmit} className="max-w-md">
            <div className="mb-4">
              <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="username">
                사용자명
              </label>
              <input
                className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
                id="username"
                type="text"
                value={userProfile.username}
                onChange={(e) => setUserProfile({...userProfile, username: e.target.value})}
                readOnly
              />
              <p className="text-xs text-gray-500 mt-1">사용자명은 변경할 수 없습니다.</p>
            </div>
            
            <div className="mb-4">
              <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="email">
                이메일
              </label>
              <input
                className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
                id="email"
                type="email"
                value={userProfile.email}
                onChange={(e) => setUserProfile({...userProfile, email: e.target.value})}
              />
            </div>
            
            <div className="mb-4">
              <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="company">
                회사
              </label>
              <input
                className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
                id="company"
                type="text"
                value={userProfile.company}
                onChange={(e) => setUserProfile({...userProfile, company: e.target.value})}
              />
            </div>
            
            <div className="mb-6">
              <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="role">
                역할
              </label>
              <input
                className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
                id="role"
                type="text"
                value={userProfile.role}
                onChange={(e) => setUserProfile({...userProfile, role: e.target.value})}
                readOnly
              />
              <p className="text-xs text-gray-500 mt-1">역할은 관리자만 변경할 수 있습니다.</p>
            </div>
            
            <div className="flex items-center justify-end">
              <button
                className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded focus:outline-none focus:shadow-outline"
                type="submit"
              >
                저장
              </button>
            </div>
          </form>
        )}
        
        {/* 비밀번호 변경 탭 */}
        {activeTab === 'password' && (
          <form onSubmit={handlePasswordSubmit} className="max-w-md">
            <div className="mb-4">
              <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="currentPassword">
                현재 비밀번호
              </label>
              <input
                className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
                id="currentPassword"
                type="password"
                placeholder="현재 비밀번호"
              />
            </div>
            
            <div className="mb-4">
              <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="newPassword">
                새 비밀번호
              </label>
              <input
                className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
                id="newPassword"
                type="password"
                placeholder="새 비밀번호"
              />
              <p className="text-xs text-gray-500 mt-1">비밀번호는 최소 8자 이상이어야 합니다.</p>
            </div>
            
            <div className="mb-6">
              <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="confirmPassword">
                비밀번호 확인
              </label>
              <input
                className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
                id="confirmPassword"
                type="password"
                placeholder="비밀번호 확인"
              />
            </div>
            
            <div className="flex items-center justify-end">
              <button
                className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded focus:outline-none focus:shadow-outline"
                type="submit"
              >
                변경
              </button>
            </div>
          </form>
        )}
        
        {/* 시스템 설정 탭 */}
        {activeTab === 'system' && (
          <form onSubmit={handleSystemSettingsSubmit} className="max-w-md">
            <div className="mb-4 flex items-center">
              <input
                id="notificationsEnabled"
                type="checkbox"
                className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                checked={systemSettings.notificationsEnabled}
                onChange={(e) => setSystemSettings({...systemSettings, notificationsEnabled: e.target.checked})}
              />
              <label htmlFor="notificationsEnabled" className="ml-2 block text-sm text-gray-900">
                알림 활성화
              </label>
            </div>
            
            <div className="mb-4 flex items-center">
              <input
                id="darkMode"
                type="checkbox"
                className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                checked={systemSettings.darkMode}
                onChange={(e) => setSystemSettings({...systemSettings, darkMode: e.target.checked})}
              />
              <label htmlFor="darkMode" className="ml-2 block text-sm text-gray-900">
                다크 모드
              </label>
            </div>
            
            <div className="mb-4 flex items-center">
              <input
                id="autoRefresh"
                type="checkbox"
                className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                checked={systemSettings.autoRefresh}
                onChange={(e) => setSystemSettings({...systemSettings, autoRefresh: e.target.checked})}
              />
              <label htmlFor="autoRefresh" className="ml-2 block text-sm text-gray-900">
                자동 새로고침
              </label>
            </div>
            
            {systemSettings.autoRefresh && (
              <div className="mb-6 pl-6">
                <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="refreshInterval">
                  새로고침 간격 (분)
                </label>
                <input
                  className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
                  id="refreshInterval"
                  type="number"
                  min="1"
                  max="60"
                  value={systemSettings.refreshInterval}
                  onChange={(e) => setSystemSettings({...systemSettings, refreshInterval: parseInt(e.target.value)})}
                />
              </div>
            )}
            
            <div className="flex items-center justify-end">
              <button
                className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded focus:outline-none focus:shadow-outline"
                type="submit"
              >
                저장
              </button>
            </div>
          </form>
        )}
        
        {/* 로그아웃 버튼 - 설정 페이지 하단에 추가 */}
        <div className="mt-8 pt-6 border-t">
          <button
            onClick={handleLogout}
            className="bg-red-500 hover:bg-red-700 text-white font-bold py-2 px-4 rounded focus:outline-none focus:shadow-outline"
          >
            로그아웃
          </button>
        </div>
      </div>
      
      <AlertModal
        isOpen={isModalOpen}
        onClose={() => setIsModalOpen(false)}
        title={modalTitle}
        message={modalMessage}
      />
    </Layout>
  );
}

export default SettingsPage;
