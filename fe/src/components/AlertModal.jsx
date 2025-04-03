import React from 'react';
import Modal from 'react-modal';

// 앱 요소 설정 (접근성을 위해 필수)
// 실제 앱에서는 main.jsx나 App.jsx에서 한 번만 설정하는 것이 좋습니다
if (typeof window !== 'undefined') {
  Modal.setAppElement('#root');
}

const customStyles = {
  overlay: {
    backgroundColor: 'rgba(0, 0, 0, 0.6)',
    zIndex: 1000
  },
  content: {
    top: '50%',
    left: '50%',
    right: 'auto',
    bottom: 'auto',
    marginRight: '-50%',
    transform: 'translate(-50%, -50%)',
    padding: '2rem',
    borderRadius: '0.5rem',
    maxWidth: '400px',
    width: '90%'
  }
};

function AlertModal({ isOpen, onClose, title, message }) {
  return (
    <Modal
      isOpen={isOpen}
      onRequestClose={onClose}
      style={customStyles}
      contentLabel="알림 모달"
    >
      <div className="text-center">
        {title && <h2 className="text-xl font-bold mb-4">{title}</h2>}
        <p className="mb-6">{message}</p>
        <button
          className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 focus:outline-none focus:ring-2 focus:ring-blue-300"
          onClick={onClose}
        >
          확인
        </button>
      </div>
    </Modal>
  );
}

export default AlertModal;
