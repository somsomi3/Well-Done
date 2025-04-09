import React from 'react';
import Header from './Header';
import Footer from './Footer';

function Layout({ children }) {
  return (
    <div className="flex flex-col h-screen overflow-hidden">
      <Header className="sticky top-0 z-50" />
      <main className="flex-grow overflow-auto">
        {children}
      </main>
      <Footer className="sticky bottom-0 z-50" />
    </div>
  );
}

export default Layout;
