import React from 'react';
import Header from './Header';
import Footer from './Footer';

// --- 중요 ---
// 아래 값들은 실제 Header와 Footer 컴포넌트의 높이에 맞게 조정해야 합니다.
// 예시: Header 높이가 Tailwind h-16 (4rem/64px), Footer 높이가 h-12 (3rem/48px)일 경우
const HEADER_HEIGHT_CLASS = 'pt-16'; // 예: 'pt-[64px]' 또는 'pt-16'
const FOOTER_HEIGHT_CLASS = 'pb-12'; // 예: 'pb-[48px]' 또는 'pb-12'
const HEADER_FIXED_HEIGHT = 'h-16'; // Header 컴포넌트에 적용될 높이 클래스 (예시)
const FOOTER_FIXED_HEIGHT = 'h-12'; // Footer 컴포넌트에 적용될 높이 클래스 (예시)

function Layout({ children }) {
  return (
    // 전체 레이아웃 컨테이너: 최소 화면 높이, Flexbox 컬럼 방향
    <div className="flex flex-col min-h-screen">
      {/* 헤더: 상단 고정 */}
      <Header
        className={`fixed top-0 left-0 w-full z-50 ${HEADER_FIXED_HEIGHT}`} // 고정 위치 및 높이 클래스 유지
      />

      {/* 메인 콘텐츠: 상하단 패딩 유지, overflow-y-auto 제거 */}
      <main
        // flex-grow: 남은 공간 모두 차지
        // pt/pb: 고정된 헤더/푸터 높이만큼 패딩 추가하여 콘텐츠 가림 방지
        // overflow-y-auto 제거됨
      >
        {children}
      </main>

      {/* 푸터: 하단 고정 */}
      <Footer
        className={`fixed bottom-0 left-0 w-full z-50 ${FOOTER_FIXED_HEIGHT}`} // 고정 위치 및 높이 클래스 유지
      />
    </div>
  );
}

export default Layout;
