import React from 'react';
import { Link } from 'react-router-dom';
import { useAuthStore } from '../../stores/authStore';
import Logout from '../Logout';
import NavItem from '../atoms/NavItem/NavItem';
import logo from '../../assets/logo.png';
import '../../styles/HeaderStyles.css';

function Header() {
  const { isAuthenticated, getUserInfo } = useAuthStore();
  const userInfo = getUserInfo();
  const isLoggedIn = isAuthenticated();

  return (
    <header className="header">
      {/* 로고 및 앱 이름 */}
      <div className="header-logo">
        <img src={logo} alt="Well-Done Logo" className="logo-image" />
        <h1 className="app-title">Well-Done</h1>
      </div>

      {/* 중앙 네비게이션 버튼 - 인증된 사용자만 표시 */}
      {isLoggedIn && (
        <nav className="header-nav">
          <NavItem to="/board">BOARD</NavItem>
          <NavItem to="/map">MAP</NavItem>
          <NavItem to="/robot">ROBOT</NavItem>
          <NavItem to="/log">LOG</NavItem>
        </nav>
      )}

      {/* 우측 버튼 영역 */}
      <div className="header-actions">
        {isLoggedIn ? (
          <>
            {/* 사용자 정보 표시 */}
            <div className="user-info">
              <span className="username">{userInfo?.username || '사용자'}</span>
            </div>

            {/* 설정 버튼 */}
            <NavItem to="/settings" className="icon-button">
              <svg
                xmlns="http://www.w3.org/2000/svg"
                className="h-6 w-6"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z"
                />
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M15 12a3 3 0 11-6 0 3 3 0 016 0z"
                />
              </svg>
            </NavItem>

            {/* 로그아웃 버튼 */}
            <Logout />
          </>
        ) : (
          // 비인증 사용자를 위한 로그인 버튼
          <Link to="/login" className="login-button">
            로그인
          </Link>
        )}
      </div>
    </header>
  );
}

export default Header;
