// StatusModal.jsx 수정 코드
import React from 'react';
import ReactModal from 'react-modal';

ReactModal.setAppElement('#root');

export default function StatusModal({ isOpen, onClose, message }) { // isOpen prop 추가
  return (
    <ReactModal
      isOpen={isOpen} // 수정된 부분
      onRequestClose={onClose}
      contentLabel="로봇 상태 모달"
      className="custom-modal"
      overlayClassName="custom-overlay"
    >
      <div className="modal-header">
        <h2>로봇 상태 안내</h2>
        <button
          onClick={onClose}
          className="close-button"
          aria-label="모달 닫기"
        >
          ✖
        </button>
      </div>
      <div className="modal-content">
        <div className="status-message">
          <div className="icon">🔋</div>
          <p>{message || "현재 상태 정보가 없습니다"}</p>
        </div>
      </div>
    </ReactModal>
  );
}
