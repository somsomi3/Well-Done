import React, { useEffect } from 'react';
import './Toast.css';
import { useToastStore } from "../../../stores/toastStore";

const Toast = ({ id, message, type }) => {
  const { removeToast } = useToastStore();

  // 자동 닫기 타이머 설정
  useEffect(() => {
    const timer = setTimeout(() => {
      removeToast(id);
    }, 5000);
    
    return () => clearTimeout(timer);
  }, [id, removeToast]);

  // 닫기 버튼 핸들러
  const handleClose = () => {
    removeToast(id);
  };

  return (
    <div className={`toast toast--${type}`}>
      <div className="toast-message">
        <p>{message}</p>
      </div>
      <button 
        className="toast-close-btn" 
        onClick={handleClose}
        aria-label="닫기"
      >
        ×
      </button>
    </div>
  );
};

export default Toast;
