import { getApiUrl } from '../configs/env';
import React, { useEffect, useRef, useState } from "react";
import Layout from "../components/Layout/Layout";
import { useAuthStore } from "../stores/authStore";
import axios from "axios";

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
  
  // 오토 맵핑 관련 상태 추가
  const [isAutoMapping, setIsAutoMapping] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [isMappingComplete, setIsMappingComplete] = useState(false);
  const [finalMapData, setFinalMapData] = useState(null);
  // 인플레이티드 맵 관련 상태 추가
  const [useInflatedMap, setUseInflatedMap] = useState(false);
  const [inflatedMapData, setInflatedMapData] = useState(null);
  
  useEffect(() => {
    if (!token) return;
    
    // 기존 맵 데이터 확인
    const checkExistingMap = async () => {
      try {
        const apiUrl = getApiUrl();
        const response = await axios.get(`${apiUrl}/robot/map-processed`, {
          headers: {
            Authorization: `Bearer ${token}`
          }
        });
        
        // 맵 데이터가 있으면 매핑이 이미 완료된 것으로 간주
        if (response.data && response.data.map) {
          console.log("기존 맵 데이터 로드:", response.data);
          setFinalMapData(response.data);
          setIsMappingComplete(true);
          
          // 인플레이티드 맵도 미리 가져오기
          fetchInflatedMap();
        }
      } catch (error) {
        console.error("기존 맵 데이터 확인 실패:", error);
      }
    };
    
    checkExistingMap();
  }, [token]);

  // Canvas에 맵 렌더링 (좌우 반전 적용)
  useEffect(() => {
    // 매핑이 완료되었다면 finalMapData를 사용, 아니면 실시간 mapData 사용
    const currentMapData = (useInflatedMap && inflatedMapData) 
                       ? inflatedMapData 
                       : (isMappingComplete ? finalMapData : mapData);
  
  // 현재 어떤 맵이 렌더링되는지 상세 로깅
    console.log("🗺️ 맵 렌더링 상태:", {
      인플레이티드맵사용중: useInflatedMap,
      맵핑완료상태: isMappingComplete,
      인플레이티드맵데이터있음: !!inflatedMapData,
      최종맵데이터있음: !!finalMapData,
      실시간맵데이터있음: !!mapData,
      현재선택된맵종류: useInflatedMap 
                  ? "인플레이티드 맵" 
                  : (isMappingComplete ? "최종 맵" : "실시간 맵"),
      맵크기: currentMapData 
            ? `${currentMapData.width}x${currentMapData.height}` 
            : "데이터 없음"
    });
    
    if (!currentMapData || !canvasRef.current) {
      console.log("⚠️ 맵 렌더링 불가 - 데이터 또는 캔버스 참조 누락");
      return;
    }
   
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const { width, height, map } = currentMapData;
   
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
   
  }, [mapData, finalMapData, inflatedMapData, isMappingComplete, useInflatedMap, pixelPosition, path]);
 
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
      // console.log("원본 메시지:", event.data);
      try {
        const data = JSON.parse(event.data);
        // console.log("파싱된 데이터:", data);
       
        // 맵핑 완료 이벤트 처리
        if (data.type === "mapping_complete") {
          console.log("🎉 맵핑 완료 알림 수신:", data);
          setIsAutoMapping(false);
          setIsLoading(false);
          setIsMappingComplete(true);
          
          // 맵핑이 완료되면 최종 맵 데이터 가져오기
          fetchFinalMap();
          
          // 알림 표시
          alert("맵핑이 완료되었습니다!");
        }
        // 매핑 중일 때만 맵 데이터 업데이트 수신
        else if (data.type === "map" && !isMappingComplete) {
          console.log("맵 데이터 수신:", data.width, "x", data.height);
          setMapData(data);
        }
        // 위치 데이터는 항상 수신 (매핑 완료 여부와 무관)
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
          // console.log("기타 데이터 수신:", data);
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
  }, [token, isMappingComplete]);

  // 맵핑 완료 후 최종 맵 데이터 가져오기
  const fetchFinalMap = async () => {
    try {
      const apiUrl = getApiUrl();
      // 일반 맵 가져오기
      const response = await axios.get(`${apiUrl}/robot/map-processed`, {
        headers: {
          Authorization: `Bearer ${token}`
        }
      });
      
      console.log("최종 맵 데이터 수신:", response.data);
      setFinalMapData(response.data);
      
      // 인플레이티드 맵도 미리 가져오기 (백그라운드에서)
      fetchInflatedMap();
    } catch (error) {
      console.error("최종 맵 데이터 가져오기 실패:", error);
      setFinalMapData(mapData);
    }
  };

  const fetchInflatedMap = async () => {
    try {
      const apiUrl = getApiUrl(); // env.js에서 API URL 가져오기
      const response = await axios.get(`${apiUrl}/robot/map-inflated`, {
        headers: {
          Authorization: `Bearer ${token}`
        }
      });
      
      console.log("인플레이티드 맵 데이터 수신:", response.data);
      setInflatedMapData(response.data);
      return true;
    } catch (error) {
      console.error("인플레이티드 맵 데이터 가져오기 실패:", error);
      alert("인플레이티드 맵을 불러오는데 실패했습니다.");
      return false;
    }
  };

  const toggleMapType = async () => {
    console.log("🔄 맵 타입 전환 요청됨. 현재 상태:", {
      현재맵타입: useInflatedMap ? "인플레이티드 맵" : "기본 맵",
      인플레이티드데이터있음: !!inflatedMapData
    });
    
    // 인플레이티드 맵으로 전환하려는데 데이터가 없는 경우
    if (!useInflatedMap && !inflatedMapData) {
      console.log("🔍 인플레이티드 맵 데이터가 없습니다. 서버에서 가져오는 중...");
      const success = await fetchInflatedMap();
      console.log(`가져오기 결과: ${success ? "성공" : "실패"}`);
      if (!success) {
        console.log("❌ 가져오기 실패로 맵 전환 취소");
        return;
      }
    }
    
    // 맵 타입 전환
    setUseInflatedMap(prev => {
      console.log(`🔄 맵 타입 전환: ${prev ? "인플레이티드 맵" : "기본 맵"}에서 ${!prev ? "인플레이티드 맵" : "기본 맵"}으로`);
      return !prev;
    });
  };

  // 오토 맵핑 시작 함수
  const startAutoMapping = async () => {
    if (isAutoMapping || isLoading) return;
    
    setIsLoading(true);
    try {
      const apiUrl = getApiUrl();
      
      // API URL에서 이미 '/api'가 포함되어 있으므로 '/robot/auto-map'만 추가
      // 이는 환경 변수 설정에 따라 달라질 수 있습니다
      const url = `${apiUrl}/robot/auto-map`;
      
      console.log("API 요청 URL:", url);  // 디버깅용
      
      const response = await axios.post(url, { data: true }, {
        headers: {
          Authorization: `Bearer ${token}`
        }
      });
      
      console.log("오토 맵핑 시작 응답:", response.data);
      setIsAutoMapping(true);
      
      setPath([]);
      setIsMappingComplete(false);
      setFinalMapData(null);
    } catch (error) {
      console.error("오토 맵핑 시작 실패:", error);
      alert("오토 맵핑 시작에 실패했습니다.");
    } finally {
      setIsLoading(false);
    }
  };

  // 오토 맵핑 중지 함수
  const stopAutoMapping = async () => {
    if (!isAutoMapping || isLoading) return;
    
    setIsLoading(true);
    try {
      const response = await axios.post('/api/robot/stop-auto-map', { data: true }, {
        headers: {
          Authorization: `Bearer ${token}`
        }
      });
      
      console.log("오토 맵핑 중지 응답:", response.data);
      setIsAutoMapping(false);
    } catch (error) {
      console.error("오토 맵핑 중지 실패:", error);
      alert("오토 맵핑 중지에 실패했습니다.");
    } finally {
      setIsLoading(false);
    }
  };
 
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
        
        {/* 오토 맵핑 제어 버튼 */}
        <div className="mb-6 flex gap-4">
          <button
            onClick={startAutoMapping}
            disabled={isAutoMapping || isLoading}
            className={`px-4 py-2 rounded font-semibold ${
              isAutoMapping
                ? 'bg-gray-300 text-gray-600 cursor-not-allowed'
                : 'bg-green-500 text-white hover:bg-green-600'
            }`}
          >
            {isLoading && !isAutoMapping ? '처리 중...' : '오토 맵핑 시작'}
          </button>
         
          <button
            onClick={stopAutoMapping}
            disabled={!isAutoMapping || isLoading}
            className={`px-4 py-2 rounded font-semibold ${
              !isAutoMapping
                ? 'bg-gray-300 text-gray-600 cursor-not-allowed'
                : 'bg-red-500 text-white hover:bg-red-600'
            }`}
          >
            {isLoading && isAutoMapping ? '처리 중...' : '오토 맵핑 중지'}
          </button>
          
          {/* 맵 타입 전환 버튼 추가 */}
          {isMappingComplete && (
            <button
              onClick={toggleMapType}
              className={`px-4 py-2 rounded font-semibold ${
                useInflatedMap
                  ? 'bg-blue-500 text-white hover:bg-blue-600'
                  : 'bg-purple-500 text-white hover:bg-purple-600'
              }`}
            >
              {useInflatedMap ? '기본 맵 보기' : '인플레이티드 맵 보기'}
            </button>
          )}
        </div>
        
        {/* 맵핑 및 뷰 상태 표시 */}
        {isAutoMapping && (
          <div className="mb-4 p-2 bg-blue-100 text-blue-800 rounded">
            🔄 오토 맵핑이 진행 중입니다...
          </div>
        )}
        
        {isMappingComplete && (
          <div className="mb-4 p-2 bg-green-100 text-green-800 rounded">
            ✅ 맵핑이 완료되었습니다! {useInflatedMap ? '인플레이티드 맵' : '기본 맵'}을 사용 중입니다.
          </div>
        )}
       
        {/* 캔버스 맵 */}
        <div style={{ position: "relative", marginBottom: "1rem" }}>
          <canvas
            ref={canvasRef}
            width={MAP_WIDTH}
            height={MAP_HEIGHT}
            style={{
              border: "1px solid #ccc",
              backgroundColor: "#f0f0f0",
              cursor: "crosshair"
            }}
            onClick={handleMapClick}
          />
         
          {/* 맵 설명 오버레이 */}
          {!mapData && !finalMapData && !inflatedMapData && (
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
        {(mapData || finalMapData || inflatedMapData) && (
          <div style={{ marginBottom: "1rem" }}>
            <h3>🗺️ 맵 정보</h3>
            <p>크기: {(useInflatedMap && inflatedMapData ? inflatedMapData : (finalMapData || mapData)).width} x {(useInflatedMap && inflatedMapData ? inflatedMapData : (finalMapData || mapData)).height} 픽셀</p>
            <p>해상도: {MAP_RESOLUTION} 미터/픽셀</p>
            <p>적용된 변환: 맵 좌우 반전, 좌표 180도 회전</p>
            {useInflatedMap && (
              <p><strong>현재 보기:</strong> 인플레이티드 맵 (장애물 주변에 안전 마진 추가됨)</p>
            )}
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
          {isMappingComplete && (
            <p>🔄 맵 보기 버튼을 클릭하여 기본 맵과 인플레이티드 맵 사이를 전환할 수 있습니다.</p>
          )}
        </div>
      </div>
    </Layout>
  );
};

export default MapPage;