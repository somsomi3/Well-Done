import React from 'react';
import { Link, useLocation } from 'react-router-dom';
import './NavItem.css';

/**
 * 네비게이션 아이템 컴포넌트
 * @param {string} to - 이동할 경로
 * @param {React.ReactNode} children - 자식 요소 (텍스트 또는 아이콘)
 * @param {string} className - 추가 클래스명
 * @param {function} onClick - 클릭 이벤트 핸들러
 */
function NavItem({ to, children, className = '', onClick }) {
  const location = useLocation();
  
  // 현재 경로가 네비게이션 아이템의 경로와 일치하는지 확인
  // 정확한 경로 일치 또는 하위 경로 일치 모두 처리
  const isActive = location.pathname === to || 
                  (to !== '/' && location.pathname.startsWith(to));
  
  return (
    <Link
      to={to}
      className={`nav-item ${isActive ? 'active' : ''} ${className}`}
      onClick={onClick}
    >
      {children}
    </Link>
  );
}

export default NavItem;
