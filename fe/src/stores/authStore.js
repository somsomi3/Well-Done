import { create } from 'zustand';

const useAuthStore = create((set) => ({
  token: localStorage.getItem('accessToken') || null,
  user: null,
  
  setToken: (token) => {
    if (token) {
      localStorage.setItem('accessToken', token);
    }
    set({ token });
  },
  
  logout: () => {
    localStorage.removeItem('accessToken');
    set({ token: null, user: null });
  },
}));

export { useAuthStore };
