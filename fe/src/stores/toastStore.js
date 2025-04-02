import { create } from 'zustand';

/**
 * 토스트 알림을 관리하는 Zustand 스토어
 */
const useToastStore = create((set, get) => ({
  // 토스트 메시지 배열
  toasts: [],
  
  /**
   * 새 토스트 메시지 추가
   * @param {Object|string} toast - 토스트 객체 또는 메시지 문자열
   * @param {string} type - 토스트 타입 (info, success, warning, error)
   * @returns {number} 생성된 토스트의 ID
   */
  addToast: (toast, type = 'info') => {
    // 문자열이 전달된 경우 토스트 객체로 변환
    const toastObj = typeof toast === 'string'
      ? { message: toast, type }
      : { ...toast, type: toast.type || type };
    
    // 고유 ID 생성
    const id = Date.now();
    const newToast = { id, ...toastObj };
    
    // 토스트 배열에 추가
    set(state => ({
      toasts: [...state.toasts, newToast]
    }));
    
    return id;
  },
  
  /**
   * 특정 ID의 토스트 제거
   * @param {number} id - 제거할 토스트의 ID
   */
  removeToast: (id) => {
    set(state => ({
      toasts: state.toasts.filter(toast => toast.id !== id)
    }));
  },
  
  /**
   * 모든 토스트 메시지 제거
   */
  clearToasts: () => {
    set({ toasts: [] });
  },
  
  /**
   * 성공 토스트 추가 (단축 메서드)
   * @param {string} message - 토스트 메시지
   * @returns {number} 생성된 토스트의 ID
   */
  success: (message) => {
    return get().addToast({ message, type: 'success' });
  },
  
  /**
   * 에러 토스트 추가 (단축 메서드)
   * @param {string} message - 토스트 메시지
   * @returns {number} 생성된 토스트의 ID
   */
  error: (message) => {
    return get().addToast({ message, type: 'error' });
  },
  
  /**
   * 경고 토스트 추가 (단축 메서드)
   * @param {string} message - 토스트 메시지
   * @returns {number} 생성된 토스트의 ID
   */
  warning: (message) => {
    return get().addToast({ message, type: 'warning' });
  },
  
  /**
   * 정보 토스트 추가 (단축 메서드)
   * @param {string} message - 토스트 메시지
   * @returns {number} 생성된 토스트의 ID
   */
  info: (message) => {
    return get().addToast({ message, type: 'info' });
  }
}));

export { useToastStore };
