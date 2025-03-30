import React from 'react';
import Header from './Header';
import Footer from './Footer';

// 레이아웃 라우팅팅
function Layout({ children }) {
  return (
    <div className="flex flex-col min-h-screen">
      <Header />
      <main className="flex-grow">
        {children}
      </main>
      <Footer />
    </div>
  );
}

export default Layout;
