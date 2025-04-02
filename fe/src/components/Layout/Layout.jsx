import React from 'react';
import Header from './Header';
import Footer from './Footer';
import ErrorBoundary from '../ErrorBoundary';

function Layout({ children }) {
  return (
    <ErrorBoundary>
      <div className="flex flex-col min-h-screen">
        <Header />
        <main className="flex-grow">
          {children}
        </main>
        <Footer />
        {/* 토스트 컨테이너는 App.jsx에서만 렌더링하므로 여기서는 제거 */}
      </div>
    </ErrorBoundary>
  );
}

export default Layout;
