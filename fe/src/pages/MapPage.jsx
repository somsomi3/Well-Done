import React, { useEffect, useRef, useState } from "react";
import Layout from "../components/Layout/Layout";
import { useAuthStore } from "../stores/authStore";

// 지도 설정 (백엔드 로그에서 확인한 값)
const MAP_WIDTH = 480;
const MAP_HEIGHT = 480;
const MAP_RESOLUTION = 0.05; // 미터/픽셀
const ORIGIN_X = -64.5; // 지도 원점 X (로봇 좌표계)
const ORIGIN_Y = -71.0; // 지도 원점 Y (로봇 좌표계)

// 로봇 좌표를 픽셀 좌표로 변환하는 함수 (180도 회전 적용)
const robotToPixelCoordinates = (robotX, robotY) => {
  // 원래 계산식
  const stdPixelX = Math.floor((robotX - ORIGIN_X) / MAP_RESOLUTION);
  const stdPixelY = MAP_HEIGHT - Math.floor((robotY - ORIGIN_Y) / MAP_RESOLUTION);
  
  // 180도 회전 적용 (x, y) -> (-x, -y) -> (width-x, height-y)
  const pixelX = MAP_WIDTH - stdPixelX;
  const pixelY = MAP_HEIGHT - stdPixelY;
  
  console.log(`변환 과정: 원본(${stdPixelX},${stdPixelY}) -> 180도 회전(${pixelX},${pixelY})`);
  
  return { pixelX, pixelY };
};

// 픽셀 좌표를 로봇 좌표로 변환하는 함수 (180도 회전 역변환)
const pixelToRobotCoordinates = (pixelX, pixelY) => {
  // 180도 회전 역변환 (동일하게 180도 회전)
  const stdPixelX = MAP_WIDTH - pixelX;
  const stdPixelY = MAP_HEIGHT - pixelY;
  
  // 로봇 좌표 계산
  const robotX = (stdPixelX * MAP_RESOLUTION) + ORIGIN_X;
  const robotY = ((MAP_HEIGHT - stdPixelY) * MAP_RESOLUTION) + ORIGIN_Y;
  
  console.log(`역변환 과정: 입력(${pixelX},${pixelY}) -> 180도 역회전(${stdPixelX},${stdPixelY}) -> 로봇(${robotX.toFixed(2)},${robotY.toFixed(2)})`);
  
  return { robotX, robotY };
};

const MapPage = () => {
  const socketRef = useRef(null);
  const canvasRef = useRef(null);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [path, setPath] = useState([]);
  const [pixelPosition, setPixelPosition] = useState({ pixelX: 0, pixelY: 0 });
  const [mapData, setMapData] = useState(null);
  const { token } = useAuthStore();
 
  // Canvas에 맵 렌더링 (좌우 반전 적용)
  useEffect(() => {
    if (!mapData || !canvasRef.current) return;
   
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const { width, height, map } = mapData;
   
    // 캔버스 클리어
    ctx.clearRect(0, 0, width, height);
   
    // 배경을 회색으로 설정 (미탐색 영역)
    ctx.fillStyle = '#e0e0e0';
    ctx.fillRect(0, 0, width, height);
   
    // 맵 데이터 렌더링 (좌우 반전 적용)
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        // 맵을 좌우로 반전 - x 좌표에서 width-1-x 값을 사용
        const value = map[y][width - 1 - x];
       
        // 값에 따라 색상 지정
        if (value === 0) {
          // 빈 공간 (탐색 완료, 이동 가능)
          ctx.fillStyle = '#ffffff';
          ctx.fillRect(x, y, 1, 1);
        } else if (value === 100) {
          // 장애물
          ctx.fillStyle = '#333333';
          ctx.fillRect(x, y, 1, 1);
        }
        // -1은 미탐색 영역으로 기본 배경색 사용
      }
    }
   
    // 경로 그리기
    if (path.length > 1) {
      ctx.beginPath();
      ctx.strokeStyle = 'blue';
      ctx.lineWidth = 2;
     
      const startPoint = robotToPixelCoordinates(path[0].x, path[0].y);
      ctx.moveTo(startPoint.pixelX, startPoint.pixelY);
     
      for (let i = 1; i < path.length; i++) {
        const point = robotToPixelCoordinates(path[i].x, path[i].y);
        ctx.lineTo(point.pixelX, point.pixelY);
      }
     
      ctx.stroke();
    }
   
    // 현재 위치 마커 그리기
    if (pixelPosition.pixelX > 0 && pixelPosition.pixelY > 0) {
      // 외부 원 (흰색 테두리)
      ctx.beginPath();
      ctx.arc(pixelPosition.pixelX, pixelPosition.pixelY, 8, 0, Math.PI * 2);
      ctx.fillStyle = 'white';
      ctx.fill();
     
      // 내부 원 (빨간색)
      ctx.beginPath();
      ctx.arc(pixelPosition.pixelX, pixelPosition.pixelY, 6, 0, Math.PI * 2);
      ctx.fillStyle = 'red';
      ctx.fill();
    }
   
  }, [mapData, pixelPosition, path]);
 
  // WebSocket 연결
  useEffect(() => {
    if (!token) return;
   
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
      console.log("원본 메시지:", event.data);
      try {
        const data = JSON.parse(event.data);
        console.log("파싱된 데이터:", data);
       
        if (data.type === "map") {
          console.log("맵 데이터 수신:", data.width, "x", data.height);
          setMapData(data);
        }
        else if (data.type === "position") {
          console.log("📍 위치 데이터 수신:", data.x, data.y);
          setPosition({ x: data.x, y: data.y });
          setPath(prev => [...prev, { x: data.x, y: data.y }]);
         
          // 로봇 좌표를 픽셀 좌표로 변환 (변환 함수 사용)
          const { pixelX, pixelY } = robotToPixelCoordinates(data.x, data.y);
          console.log("변환된 픽셀 좌표:", pixelX, pixelY);
          setPixelPosition({ pixelX, pixelY });
        }
        else if (data.x !== undefined && data.y !== undefined && !data.type) {
          console.log("📍 기존 형식 위치 데이터 수신:", data.x, data.y);
          setPosition(data);
          setPath(prev => [...prev, data]);
         
          const { pixelX, pixelY } = robotToPixelCoordinates(data.x, data.y);
          console.log("변환된 픽셀 좌표:", pixelX, pixelY);
          setPixelPosition({ pixelX, pixelY });
        }
        else {
          console.log("기타 데이터 수신:", data);
        }
      } catch (error) {
        console.error("데이터 파싱 오류:", error);
      }
    };
   
    socket.onclose = () => {
      console.log("❌ WebSocket 종료");
    };
   
    return () => {
      socket.close();
    };
  }, [token]);
 
  // 맵 클릭 이벤트 처리 (테스트용)
  const handleMapClick = (e) => {
    if (!canvasRef.current) return;
   
    const rect = canvasRef.current.getBoundingClientRect();
    const pixelX = Math.floor(e.clientX - rect.left);
    const pixelY = Math.floor(e.clientY - rect.top);
   
    // 픽셀 좌표를 로봇 좌표로 변환 (역변환 함수 사용)
    const { robotX, robotY } = pixelToRobotCoordinates(pixelX, pixelY);
   
    console.log("맵 클릭 - 픽셀:", pixelX, pixelY, "로봇:", robotX.toFixed(2), robotY.toFixed(2));
   
    // 위치 및 경로 업데이트
    const newPosition = { x: robotX, y: robotY };
    setPosition(newPosition);
    setPath(prev => [...prev, newPosition]);
    setPixelPosition({ pixelX, pixelY });
  };
 
  return (
    <Layout>
      <div style={{ fontFamily: "Arial", padding: "1rem" }}>
        <h2>📍 실시간 로봇 위치 보기 (맵 좌우 반전 및 좌표 180도 회전)</h2>
       
        {/* 캔버스 맵 */}
        <div style={{ position: "relative", marginBottom: "1rem" }}>
          <canvas
            ref={canvasRef}
            width={MAP_WIDTH}
            height={MAP_HEIGHT}
            style={{
              border: "1px solid #ccc",
              backgroundColor: "#f0f0f0", // 기본 배경색
              cursor: "crosshair" // 클릭 가능함을 나타내는 커서
            }}
            onClick={handleMapClick}
          />
         
          {/* 맵 설명 오버레이 */}
          {!mapData && (
            <div style={{
              position: "absolute",
              top: "50%",
              left: "50%",
              transform: "translate(-50%, -50%)",
              textAlign: "center",
              color: "#666",
              pointerEvents: "none"
            }}>
              맵 데이터 로딩 중...
            </div>
          )}
        </div>
       
        {/* 좌표 정보 표시 */}
        <p>
          <strong>로봇 좌표:</strong> X: {position.x.toFixed(2)}, Y: {position.y.toFixed(2)}
        </p>
        <p>
          <strong>픽셀 좌표:</strong> X: {pixelPosition.pixelX}, Y: {pixelPosition.pixelY}
        </p>
       
        {/* 맵 정보 */}
        {mapData && (
          <div style={{ marginBottom: "1rem" }}>
            <h3>🗺️ 맵 정보</h3>
            <p>크기: {mapData.width} x {mapData.height} 픽셀</p>
            <p>해상도: {MAP_RESOLUTION} 미터/픽셀</p>
            <p>적용된 변환: 맵 좌우 반전, 좌표 180도 회전</p>
          </div>
        )}
       
        {/* 경로 기록 */}
        <h3>🛤 이동 경로</h3>
        <div
          style={{
            maxHeight: "150px",
            overflowY: "auto",
            backgroundColor: "#f9f9f9",
            padding: "0.5rem",
            width: "400px",
          }}
        >
          {path.length === 0 ? (
            <p>아직 기록된 경로가 없습니다. 맵을 클릭하여 위치를 시뮬레이션할 수 있습니다.</p>
          ) : (
            <ul>
              {path.map((pos, index) => (
                <li key={index}>
                  #{index + 1} → X: {pos.x.toFixed(2)}, Y: {pos.y.toFixed(2)}
                </li>
              ))}
            </ul>
          )}
        </div>
       
        {/* 도움말 */}
        <div style={{ marginTop: "1rem", fontSize: "0.9rem", color: "#666" }}>
          <p>💡 맵을 클릭하여 로봇 위치를 시뮬레이션할 수 있습니다.</p>
          <p>⚪ 회색: 미탐색 영역 / ⚫ 검은색: 장애물 / ⚪ 흰색: 이동 가능 영역</p>
          <p>ℹ️ 맵은 좌우로 반전되었고, 로봇 좌표는 180도 회전되어 표시됩니다.</p>
        </div>
      </div>
    </Layout>
  );
};

export default MapPage;