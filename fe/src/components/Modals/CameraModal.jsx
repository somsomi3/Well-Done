import React, { useEffect, useState, useRef } from "react";
import ReactModal from 'react-modal';
import { useAuthStore } from '../../stores/authStore';

// 이미지 새로고침 주기 (밀리초)
const IMAGE_REFRESH_INTERVAL = 1000;

const CameraModal = ({ onClose, robotId }) => {
  const [imageData, setImageData] = useState(null);
  const [lastUpdated, setLastUpdated] = useState(null);
  const [imageError, setImageError] = useState(false);
  const [loading, setLoading] = useState(true);
  const socketRef = useRef(null);
  const fetchIntervalRef = useRef(null);
  const { token } = useAuthStore();

  // WebSocket 연결 초기화 (모달 열릴 때만 실행)
  useEffect(() => {
    if (!token) return;

    const socket = new WebSocket(
      `wss://j12e102.p.ssafy.io/ws/user?token=${token}`
    );
    socketRef.current = socket;

    socket.onopen = () => {
      console.log("WebSocket 연결됨");
      socket.send(JSON.stringify({ type: "join" }));
    };

    socket.onmessage = (event) => {
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

    return () => {
      socket.close();
      console.log("❌ WebSocket 종료");
    };
  }, [token]); // 모달이 열릴 때마다 토큰 갱신

  // HTTP 폴링 초기화
  useEffect(() => {
    if (!token) return;

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
    fetchIntervalRef.current = setInterval(fetchCameraImage, IMAGE_REFRESH_INTERVAL);

    return () => clearInterval(fetchIntervalRef.current);
  }, [token]);

  return (
    <ReactModal
      isOpen={true}
      onRequestClose={onClose}
      contentLabel="카메라 모달"
      className="custom-modal"
    >
      <div className="p-4">
        <h2 className="text-xl font-bold mb-4">로봇 #{robotId} 카메라</h2>
        {/* 이미지 표시 로직 유지 */}
        <button 
          onClick={onClose}
          className="mt-4 px-4 py-2 bg-red-500 text-white rounded"
        >
          닫기
        </button>
      </div>
    </ReactModal>
  );
};

export default CameraModal;
