import React from 'react';

function Footer() {
  const currentYear = new Date().getFullYear();

  return (
    <footer className="bg-gray-800 text-white h-[10vh] flex items-center justify-center">
      <div className="text-center">
        <p>© {currentYear} 로봇 관리 시스템. All rights reserved.</p>
        <p className="text-sm text-gray-400">버전 1.0.0</p>
      </div>
    </footer>
  );
}

export default Footer;
