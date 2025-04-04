import { useEffect, useState } from 'react';
import { useLocation as useRouterLocation } from 'react-router-dom';

/**
 * 현재 경로를 추적하고 경로 변경 시 추가 작업을 수행하는 커스텀 훅
 * @param {function} onLocationChange - 경로 변경 시 호출될 콜백 함수 (선택적)
 * @returns {object} location - React Router의 location 객체
 */
function useLocation(onLocationChange) {
  const location = useRouterLocation();
  const [prevPathname, setPrevPathname] = useState(location.pathname);

  useEffect(() => {
    // 경로가 변경되었을 때만 처리
    if (location.pathname !== prevPathname) {
      // 이전 경로 업데이트
      setPrevPathname(location.pathname);
      
      // 콜백 함수가 제공된 경우 호출
      if (typeof onLocationChange === 'function') {
        onLocationChange(location, prevPathname);
      }
      
      // 페이지 제목 업데이트 (선택적)
      updatePageTitle(location.pathname);
      
      // 페이지 상단으로 스크롤 (선택적)
      window.scrollTo(0, 0);
    }
  }, [location, prevPathname, onLocationChange]);

  return location;
}

/**
 * 경로에 따라 페이지 제목 업데이트
 * @param {string} pathname - 현재 경로
 */
function updatePageTitle(pathname) {
  const baseTitle = 'Well-Done';
  let pageTitle = '';

  // 경로에 따라 페이지 제목 설정
  switch (pathname) {
    case '/':
    case '/main':
      pageTitle = '메인';
      break;
    case '/map':
      pageTitle = '지도';
      break;
    case '/robot':
      pageTitle = '로봇';
      break;
    case '/log':
      pageTitle = '로그';
      break;
    case '/settings':
      pageTitle = '설정';
      break;
    case '/login':
      pageTitle = '로그인';
      break;
    case '/register':
      pageTitle = '회원가입';
      break;
    default:
      // 경로가 /some-path/123 형태인 경우 기본 경로 추출
      const basePath = pathname.split('/')[1];
      if (basePath) {
        pageTitle = basePath.charAt(0).toUpperCase() + basePath.slice(1);
      }
  }

  // 페이지 제목 설정
  document.title = pageTitle ? `${pageTitle} | ${baseTitle}` : baseTitle;
}

export default useLocation;
