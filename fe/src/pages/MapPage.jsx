import React, { useEffect, useRef, useState } from "react";
import Layout from "../components/Layout/Layout";
import { useAuthStore } from "../stores/authStore"; // ← 경로 확인!!

// console.log("📍 현재 위치 수신:", data);

// 좌표 매핑용 지도 범위 (로봇 좌표 기준)
const mapBounds = {
  xMin: -100,
  xMax: 100,
  yMin: -100,
  yMax: 100,
};
// x, y → 퍼센트 좌표로 변환
const mapToPercentage = (x, y) => {
  const mappedX =
    ((x - mapBounds.xMin) / (mapBounds.xMax - mapBounds.xMin)) * 100;
  const mappedY =
    ((y - mapBounds.yMin) / (mapBounds.yMax - mapBounds.yMin)) * 100;
  return { mappedX, mappedY };
};

// 페이지 안에 포함된 Viewer 컴포넌트
const MapPage = () => {
  const socketRef = useRef(null);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [path, setPath] = useState([]);
  const { token } = useAuthStore();
  const [mapData, setMapData] = useState(null);

  useEffect(() => {
    if (!token) return;

    // const socket = new WebSocket(`ws://localhost:8080/ws/user?token=${token}`);
    const socket = new WebSocket(
      `wss://j12e102.p.ssafy.io/ws/user?token=${token}`
    );

    socketRef.current = socket;

    socket.onopen = () => {
      console.log("WebSocket 연결됨");
      socket.send(
        JSON.stringify({
          type: "join",
          // user_id: userId,
        })
      );
    };

    socket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      console.log("📍 현재 위치 수신:", data);

      if (data.type === "map") {
        console.log("🗺️ 실시간 맵 수신됨:", data.map);
        setMapData({
          map: data.map,
          width: data.width,
          height: data.height,
        });
      }

      if (data.x !== undefined && data.y !== undefined) {
        setPosition(data);
        setPath((prev) => [...prev, data]);
      }
    };

    socket.onclose = () => {
      console.log("❌ WebSocket 종료");
    };

    return () => {
      socket.close();
    };
  }, [token]);

  const { mappedX, mappedY } = mapToPercentage(position.x, position.y);

  return (
    <Layout>
      <div style={{ fontFamily: "Arial", padding: "1rem" }}>
        <h2>📍 실시간 로봇 위치 보기</h2>

        {/* 지도 이미지 + 마커 */}
        <div
          style={{
            position: "relative",
            width: "480px",
            height: "480px",
            border: "1px solid #ccc",
            marginBottom: "1rem",
          }}
        >
          {/* 지도 이미지 */}
          <img
            src="/map-image.png"
            alt="Map"
            style={{ width: "100%", height: "100%" }}
          />

          {/* 마커 (현재 위치) */}
          <div
            style={{
              position: "absolute",
              top: `${100 - mappedY}%`, // Y는 상하 반전 필요
              left: `${mappedX}%`,
              transform: "translate(-50%, -50%)",
              width: "14px",
              height: "14px",
              backgroundColor: "red",
              borderRadius: "50%",
              border: "2px solid white",
              boxShadow: "0 0 5px black",
            }}
          />
        </div>

        {/* 현재 좌표 출력 */}
        <p>
          현재 위치: <strong>X:</strong> {position.x.toFixed(2)},{" "}
          <strong>Y:</strong> {position.y.toFixed(2)}
        </p>

        {/* 경로 출력 */}
        <h3>🛤 이동 경로</h3>
        <ul
          style={{
            maxHeight: "150px",
            overflowY: "auto",
            backgroundColor: "#f9f9f9",
            padding: "0.5rem",
            width: "400px",
          }}
        >
          {path.map((pos, index) => (
            <li key={index}>
              #{index + 1} → X: {pos.x.toFixed(2)}, Y: {pos.y.toFixed(2)}
            </li>
          ))}
        </ul>

        {/* 여기에 맵 렌더링 추가 */}
        {mapData && (
          <div style={{ marginTop: "2rem" }}>
            <h3>🗺️ 실시간 맵</h3>
            <div
              style={{
                display: "grid",
                gridTemplateColumns: `repeat(${mapData.width}, 4px)`,
                gridTemplateRows: `repeat(${mapData.height}, 4px)`,
                gap: "1px",
                backgroundColor: "#ddd",
              }}
            >
              {mapData.map.flat().map((value, idx) => (
                <div
                  key={idx}
                  style={{
                    width: "4px",
                    height: "4px",
                    backgroundColor:
                      value === 100
                        ? "#333"
                        : value === 0
                        ? "#eee"
                        : "transparent",
                  }}
                />
              ))}
            </div>
          </div>
        )}
      </div>
    </Layout>
  );
};

export default MapPage;
