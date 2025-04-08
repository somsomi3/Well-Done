import React, { useEffect, useState, useRef } from "react";
import ReactModal from 'react-modal';
import { useAuthStore } from '../../stores/authStore';

// ReactModal 설정: 접근성 및 기본 스타일 설정
ReactModal.setAppElement('#root');

const IMAGE_REFRESH_INTERVAL = 1000;

const CameraModal = ({ isOpen, onClose, robotId }) => {
  const [imageData, setImageData] = useState(null);
  const [lastUpdated, setLastUpdated] = useState(null);
  const [imageError, setImageError] = useState(false);
  const [loading, setLoading] = useState(true);
  const socketRef = useRef(null);
  const fetchIntervalRef = useRef(null);
  const { token } = useAuthStore();

  // WebSocket 연결 관리
  useEffect(() => {
    if (!token || !isOpen) return;

    const socket = new WebSocket(
      `wss://j12e102.p.ssafy.io/ws/user?token=${token}`
    );
    socketRef.current = socket;

    const handleMessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.type === "camera_image" && data.data) {
          setImageData(data.data);
          setLastUpdated(new Date());
          setLoading(false);
          setImageError(false);
        }
      } catch (error) {
        console.error("데이터 파싱 오류:", error);
      }
    };

    socket.addEventListener('open', () => {
      console.log("WebSocket 연결됨");
      socket.send(JSON.stringify({ type: "join" }));
    });

    socket.addEventListener('message', handleMessage);

    return () => {
      if (socket.readyState === 1) {
        socket.close();
        console.log("❌ WebSocket 안전하게 종료");
      }
      socket.removeEventListener('message', handleMessage);
    };
  }, [token, isOpen]);

  // HTTP 폴링 관리
  useEffect(() => {
    if (!token || !isOpen) return;

    const fetchCameraImage = async () => {
      try {
        const response = await fetch(
          "https://j12e102.p.ssafy.io/api/robot/image-jpeg-compressed",
          { headers: { Authorization: `Bearer ${token}` } }
        );

        if (!response.ok) throw new Error(`HTTP 오류: ${response.status}`);
        const data = await response.json();
 
        if (data?.data) {
          setImageData(data.data);
          setLastUpdated(new Date());
          setLoading(false);
          setImageError(false);
        }
      } catch (error) {
        console.error("이미지 가져오기 오류:", error);
        setImageError(true);
      }
    };

    fetchCameraImage();
    const interval = setInterval(fetchCameraImage, IMAGE_REFRESH_INTERVAL);
    return () => clearInterval(interval);
  }, [token, isOpen]);

  return (
    <ReactModal
      isOpen={isOpen}
      onRequestClose={onClose}
      contentLabel="카메라 모달"
      className="custom-modal"
      overlayClassName="custom-overlay"
      shouldCloseOnOverlayClick={true}
      shouldCloseOnEsc={true}
    >
      <div className="p-4">
        <h2 className="text-xl font-bold mb-4">로봇 #{robotId} 카메라 화면</h2>

        {/* 이미지 표시 영역 */}
        <div className="relative w-[640px] h-[480px] bg-gray-100 border border-gray-300">
          {loading ? (
            <div className="text-center text-gray-600">로딩 중...</div>
          ) : imageError ? (
            <div className="text-center text-red-500">연결 오류</div>
          ) : imageData ? (
            <img
              src={`data:image/jpeg;base64,${imageData}`}
              alt="로봇 카메라 화면"
              className="w-full h-full object-contain"
            />
          ) : (
            <div className="text-center text-gray-600">데이터 없음</div>
          )}
        </div>

        {/* 닫기 버튼 */}
        <button
          onClick={onClose}
          className="mt-4 px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 transition-colors"
        >
          닫기
        </button>
      </div>
    </ReactModal>
  );
};

export default CameraModal;
