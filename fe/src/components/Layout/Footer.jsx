import React from 'react';

function Footer({ className }) {
  return (
    <footer className={`bg-gray-800 text-white py-4 ${className}`}>
      <div className="container mx-auto px-4">
        <div className="flex flex-col md:flex-row justify-between items-center">
          <div className="text-sm">
            &copy; {new Date().getFullYear()} Well-Done. All rights reserved.
          </div>
          <div className="text-sm mt-2 md:mt-0">
            <span>버전: 1.0.0</span>
            <span className="mx-2">|</span>
            <span>문의: support@well-done.com</span>
          </div>
        </div>
      </div>
    </footer>
  );
}

export default Footer;
