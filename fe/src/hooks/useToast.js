import { useCallback } from 'react';
import { useToastStore } from '../stores/toastStore';

export const useToast = () => {
  const { 
    addToast: addToastToStore, 
    removeToast, 
    success: successToast,
    error: errorToast,
    warning: warningToast,
    info: infoToast
  } = useToastStore();

  // 기본 토스트 추가 함수
  const addToast = useCallback((message, type = 'info', duration = 5000) => {
    const id = addToastToStore({ message, type });

    if (duration > 0) {
      setTimeout(() => removeToast(id), duration);
    }

    return id;
  }, [addToastToStore, removeToast]);

  // 성공 토스트 (단축 메서드)
  const success = useCallback((message, duration = 5000) => {
    const id = successToast(message);
    if (duration > 0) {
      setTimeout(() => removeToast(id), duration);
    }
    return id;
  }, [successToast, removeToast]);

  // 에러 토스트 (단축 메서드)
  const error = useCallback((message, duration = 5000) => {
    const id = errorToast(message);
    if (duration > 0) {
      setTimeout(() => removeToast(id), duration);
    }
    return id;
  }, [errorToast, removeToast]);

  // 경고 토스트 (단축 메서드)
  const warning = useCallback((message, duration = 5000) => {
    const id = warningToast(message);
    if (duration > 0) {
      setTimeout(() => removeToast(id), duration);
    }
    return id;
  }, [warningToast, removeToast]);

  // 정보 토스트 (단축 메서드)
  const info = useCallback((message, duration = 5000) => {
    const id = infoToast(message);
    if (duration > 0) {
      setTimeout(() => removeToast(id), duration);
    }
    return id;
  }, [infoToast, removeToast]);

  return { 
    addToast, 
    removeToast,
    success,
    error,
    warning,
    info
  };
};
