import React from 'react';
import { Link, useNavigate } from 'react-router-dom';
import { useAuthStore } from '../../stores/authStore';
import Logout from '../Logout';

function Header() {
  const navigate = useNavigate();
  const { isAuthenticated, getUserInfo } = useAuthStore();
  const userInfo = getUserInfo();

  return (
    <header className="bg-blue-600 text-white h-[10vh] flex items-center justify-between px-4">
      {/* 로고 또는 왼쪽 영역 */}
      <div className="flex items-center">
        <h1 className="text-xl font-bold">로봇 관리 시스템</h1>
      </div>

      {/* 중앙 네비게이션 버튼 - 인증된 사용자만 표시 */}
      {isAuthenticated() && (
        <nav className="flex items-center space-x-4">
          <Link to="/main" className="px-4 py-2 rounded hover:bg-blue-700 transition-colors">
            MAIN
          </Link>
          <Link to="/map" className="px-4 py-2 rounded hover:bg-blue-700 transition-colors">
            MAP
          </Link>
          <Link to="/robot" className="px-4 py-2 rounded hover:bg-blue-700 transition-colors">
            ROBOT
          </Link>
          <Link to="/log" className="px-4 py-2 rounded hover:bg-blue-700 transition-colors">
            LOG
          </Link>
        </nav>
      )}

      {/* 우측 버튼 영역 */}
      <div className="flex items-center space-x-2">
        {isAuthenticated() ? (
          <>
            {/* 사용자 정보 표시 */}
            <div className="mr-4 text-sm">
              <span className="font-medium">{userInfo?.username || '사용자'}</span>
            </div>
            
            {/* 설정 버튼 */}
            <button 
              className="p-2 rounded hover:bg-blue-700 transition-colors"
              onClick={() => navigate('/settings')}
              title="설정"
            >
              <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z" />
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
              </svg>
            </button>
            
            {/* 로그아웃 버튼 */}
            <Logout />
          </>
        ) : (
          // 비인증 사용자를 위한 로그인 버튼
          <button
            className="bg-white text-blue-600 hover:bg-gray-100 px-4 py-2 rounded font-medium"
            onClick={() => navigate('/login')}
          >
            로그인
          </button>
        )}
      </div>
    </header>
  );
}

export default Header;
