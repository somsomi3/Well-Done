import React from 'react';
import { useAuthStore } from '../stores/authStore';

function Logout() {
  const { logout } = useAuthStore();

  const handleLogout = async () => {
    await logout();
    window.location.href = '/login';
  };

  return (
    <button
      className="bg-red-500 hover:bg-red-700 text-white font-bold py-2 px-4 rounded focus:outline-none focus:shadow-outline"
      onClick={handleLogout}
    >
      로그아웃
    </button>
  );
}

export default Logout;
