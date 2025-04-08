import React, { useEffect, useState, useRef } from "react";
import Layout from "../components/Layout/Layout";
import { useAuthStore } from "../stores/authStore";

// 이미지 새로고침 주기 (밀리초)
const IMAGE_REFRESH_INTERVAL = 1000;

const CameraViewPage = () => {
  const [imageData, setImageData] = useState(null);
  const [lastUpdated, setLastUpdated] = useState(null);
  const [imageError, setImageError] = useState(false);
  const [loading, setLoading] = useState(true);
  const socketRef = useRef(null);
  const fetchIntervalRef = useRef(null);
  const { token } = useAuthStore();

  // WebSocket 연결 초기화
  useEffect(() => {
    if (!token) return;

    // WebSocket 연결
    const socket = new WebSocket(
      `wss://j12e102.p.ssafy.io/ws/user?token=${token}`
    );
    socketRef.current = socket;

    socket.onopen = () => {
      console.log("WebSocket 연결됨");
      socket.send(
        JSON.stringify({
          type: "join",
        })
      );
    };

    socket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);

        // 이미지 데이터가 WebSocket으로 전송되는 경우 처리
        if (data.type === "camera_image" && data.data) {
          console.log("WebSocket으로 카메라 이미지 수신");
          setImageData(data.data);
          setLastUpdated(new Date());
          setLoading(false);
          setImageError(false);
        }
      } catch (error) {
        console.error("데이터 파싱 오류:", error);
      }
    };

    socket.onclose = () => {
      console.log("❌ WebSocket 종료");
    };

    // 컴포넌트 언마운트 시 WebSocket 연결 종료
    return () => {
      socket.close();
    };
  }, [token]);

  // HTTP 폴링 방식으로 이미지 정기적으로 가져오기
  useEffect(() => {
    if (!token) return;

    // 즉시 첫 번째 이미지 가져오기
    fetchCameraImage();

    // 주기적으로 이미지 가져오기
    fetchIntervalRef.current = setInterval(
      fetchCameraImage,
      IMAGE_REFRESH_INTERVAL
    );

    // 컴포넌트 언마운트 시 인터벌 정리
    return () => {
      if (fetchIntervalRef.current) {
        clearInterval(fetchIntervalRef.current);
      }
    };
  }, [token]);

  // 카메라 이미지 가져오기 함수
  const fetchCameraImage = async () => {
    if (!token) return;

    try {
      const response = await fetch(
        "https://j12e102.p.ssafy.io/api/robot/image-jpeg-compressed",
        {
          headers: {
            Authorization: `Bearer ${token}`,
          },
        }
      );

      if (!response.ok) {
        throw new Error(`HTTP 오류: ${response.status}`);
      }

      const data = await response.json();

      // data.data가 있는지 확인
      if (data && data.data) {
        setImageData(data.data);
        setLastUpdated(new Date());
        setLoading(false);
        setImageError(false);
      } else {
        console.warn("이미지 데이터가 없습니다");
        setImageError(true);
      }
    } catch (error) {
      console.error("이미지 가져오기 오류:", error);
      setImageError(true);
    }
  };

  // 이미지 수동 새로고침 함수
  const handleRefresh = () => {
    setLoading(true);
    fetchCameraImage();
  };

  return (
    <Layout>
      <div style={{ fontFamily: "Arial", padding: "1rem" }}>
        <h2>📹 실시간 로봇 카메라 화면</h2>

        {/* 이미지 표시 영역 */}
        <div
          style={{
            position: "relative",
            marginBottom: "1rem",
            border: "1px solid #ccc",
            backgroundColor: "#f0f0f0",
            height: "480px",
            width: "640px",
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            overflow: "hidden",
          }}
        >
          {loading ? (
            <div style={{ textAlign: "center", color: "#666" }}>
              카메라 이미지 로딩 중...
            </div>
          ) : imageError ? (
            <div style={{ textAlign: "center", color: "#ff4444" }}>
              카메라 이미지를 불러올 수 없습니다.
            </div>
          ) : imageData ? (
            <img
              src={`data:image/jpeg;base64,${imageData}`}
              alt="로봇 카메라 화면"
              style={{
                maxWidth: "100%",
                maxHeight: "100%",
                objectFit: "contain",
              }}
            />
          ) : (
            <div style={{ textAlign: "center", color: "#666" }}>
              카메라 이미지가 없습니다.
            </div>
          )}
        </div>

        {/* 이미지 정보 및 컨트롤 */}
        <div style={{ marginBottom: "1rem" }}>
          <div style={{ display: "flex", alignItems: "center", gap: "1rem" }}>
            <button
              onClick={handleRefresh}
              style={{
                padding: "0.5rem 1rem",
                backgroundColor: "#4682B4",
                color: "white",
                border: "none",
                borderRadius: "4px",
                cursor: "pointer",
              }}
            >
              🔄 새로고침
            </button>

            {lastUpdated && (
              <p style={{ margin: 0 }}>
                마지막 업데이트: {lastUpdated.toLocaleTimeString()}
              </p>
            )}
          </div>
        </div>

        {/* 상태 정보 */}
        <div style={{ marginBottom: "1rem" }}>
          <h3>📊 연결 상태</h3>
          <div
            style={{
              display: "flex",
              gap: "1rem",
              padding: "0.5rem",
              backgroundColor: "#f9f9f9",
            }}
          >
            <div
              style={{
                display: "flex",
                alignItems: "center",
                gap: "0.5rem",
              }}
            >
              <span
                style={{
                  width: "12px",
                  height: "12px",
                  borderRadius: "50%",
                  backgroundColor:
                    socketRef.current?.readyState === 1 ? "#4CAF50" : "#FF5722",
                  display: "inline-block",
                }}
              ></span>
              <span>
                WebSocket:{" "}
                {socketRef.current?.readyState === 1 ? "연결됨" : "연결 안됨"}
              </span>
            </div>

            <div
              style={{
                display: "flex",
                alignItems: "center",
                gap: "0.5rem",
              }}
            >
              <span
                style={{
                  width: "12px",
                  height: "12px",
                  borderRadius: "50%",
                  backgroundColor: !imageError ? "#4CAF50" : "#FF5722",
                  display: "inline-block",
                }}
              ></span>
              <span>이미지 데이터: {!imageError ? "정상" : "오류"}</span>
            </div>
          </div>
        </div>

        {/* 도움말 */}
        <div style={{ marginTop: "1rem", fontSize: "0.9rem", color: "#666" }}>
          <p>
            💡 로봇의 카메라 화면이 {IMAGE_REFRESH_INTERVAL / 1000}초마다
            자동으로 업데이트됩니다.
          </p>
          <p>
            🔄 새로고침 버튼을 클릭하여 수동으로 이미지를 업데이트할 수
            있습니다.
          </p>
          <p>
            ℹ️ 이미지가 보이지 않는 경우, 로봇의 카메라 상태를 확인해 주세요.
          </p>
        </div>
      </div>
    </Layout>
  );
};

export default CameraViewPage;
