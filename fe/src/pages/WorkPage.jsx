import { getApiUrl } from "../configs/env";
import React, { useEffect, useRef, useState } from "react";
import Layout from "../components/Layout/Layout";
import { useAuthStore } from "../stores/authStore";
import axios from "axios";

// 지도 설정 (기존 MapPage와 동일)
const MAP_WIDTH = 480;
const MAP_HEIGHT = 480;
const MAP_RESOLUTION = 0.05; // 미터/픽셀
const ORIGIN_X = -64.5; // 지도 원점 X (로봇 좌표계)
const ORIGIN_Y = -71.0; // 지도 원점 Y (로봇 좌표계)

// 로봇 좌표를 픽셀 좌표로 변환하는 함수 (180도 회전 적용)
const robotToPixelCoordinates = (robotX, robotY) => {
  // 원래 계산식
  const stdPixelX = Math.floor((robotX - ORIGIN_X) / MAP_RESOLUTION);
  const stdPixelY =
    MAP_HEIGHT - Math.floor((robotY - ORIGIN_Y) / MAP_RESOLUTION);

  // 180도 회전 적용 (x, y) -> (-x, -y) -> (width-x, height-y)
  const pixelX = MAP_WIDTH - stdPixelX;
  const pixelY = MAP_HEIGHT - stdPixelY;

  return { pixelX, pixelY };
};

// 물품 종류 정의
const PRODUCTS = [
  { id: "쿠크다스", name: "쿠크다스" },
  { id: "몽쉘", name: "몽쉘" },
];

// 위치 데이터 정의
const LOCATIONS = [
  // 랙 위치
  { id: "A1", name: "랙 A1", x: -47.36, y: -63.7, theta: 180 },
  { id: "A2", name: "랙 A2", x: -47.89, y: -63.69, theta: 180 },
  { id: "B1", name: "랙 B1", x: -51.16, y: -63.69, theta: 180 },
  { id: "B2", name: "랙 B2", x: -51.67, y: -63.69, theta: 180 },
  { id: "C1", name: "랙 C1", x: -48.73, y: -63.07, theta: 90 },
  { id: "C2", name: "랙 C2", x: -48.7, y: -62.58, theta: 90 },
  { id: "D1", name: "랙 D1", x: -50.45, y: -63.11, theta: -90 },
  { id: "D2", name: "랙 D2", x: -50.45, y: -62.56, theta: -90 },
  { id: "E1", name: "랙 E1", x: -47.38, y: -61.98, theta: 0 },
  { id: "E2", name: "랙 E2", x: -47.89, y: -61.98, theta: 0 },
  { id: "F1", name: "랙 F1", x: -51.17, y: -62.01, theta: 0 },
  { id: "F2", name: "랙 F2", x: -51.69, y: -61.93, theta: 0 },
  { id: "G1", name: "랙 G1", x: -47.4, y: -60.27, theta: 180 },
  { id: "G2", name: "랙 G2", x: -47.9, y: -60.25, theta: 180 },
  { id: "H1", name: "랙 H1", x: -51.2, y: -60.17, theta: 180 },
  { id: "H2", name: "랙 H2", x: -51.7, y: -60.19, theta: 180 },
  { id: "I1", name: "랙 I1", x: -48.75, y: -59.4, theta: 90 },
  { id: "I2", name: "랙 I2", x: -48.73, y: -58.92, theta: 90 },
  { id: "J1", name: "랙 J1", x: -50.45, y: -59.41, theta: -90 },
  { id: "J2", name: "랙 J2", x: -50.45, y: -58.9, theta: -90 },
  { id: "K1", name: "랙 K1", x: -47.39, y: -58.01, theta: 0 },
  { id: "K2", name: "랙 K2", x: -47.9, y: -58.01, theta: 0 },
  { id: "L1", name: "랙 L1", x: -51.26, y: -57.97, theta: 0 },
  { id: "L2", name: "랙 L2", x: -51.75, y: -57.97, theta: 0 },

  // 빈 파레트 저장소
  { id: "EMP1", name: "빈 파레트 1", x: -55.56, y: -66.42, theta: 0 },
  { id: "EMP2", name: "임시 저장소", x: -55.03, y: -66.42, theta: 0 },

  // 창고 왼쪽 줄(몽쉘)
  { id: "LST1", name: "창고 왼쪽 1", x: -59.2, y: -64.82, theta: -90 },
  { id: "LST2", name: "창고 왼쪽 2", x: -59.2, y: -64.32, theta: -90 },
  { id: "LST3", name: "창고 왼쪽 3", x: -59.2, y: -63.82, theta: -90 },
  { id: "LST4", name: "창고 왼쪽 4", x: -59.2, y: -63.32, theta: -90 },
  { id: "LST5", name: "창고 왼쪽 5", x: -59.2, y: -62.82, theta: -90 },
  { id: "LST6", name: "창고 왼쪽 6", x: -59.2, y: -62.32, theta: -90 },
  { id: "LST7", name: "창고 왼쪽 7", x: -59.2, y: -61.82, theta: -90 },
  { id: "LST8", name: "창고 왼쪽 8", x: -59.2, y: -61.32, theta: -90 },
  { id: "LST9", name: "창고 왼쪽 9", x: -59.2, y: -60.82, theta: -90 },
  { id: "LST10", name: "창고 왼쪽 10", x: -59.2, y: -60.32, theta: -90 },

  // 창고 오른쪽 줄(쿠크다스)
  { id: "RST1", name: "창고 오른쪽 1", x: -60.19, y: -64.82, theta: 90 },
  { id: "RST2", name: "창고 오른쪽 2", x: -60.19, y: -64.32, theta: 90 },
  { id: "RST3", name: "창고 오른쪽 3", x: -60.19, y: -63.82, theta: 90 },
  { id: "RST4", name: "창고 오른쪽 4", x: -60.19, y: -63.32, theta: 90 },
  { id: "RST5", name: "창고 오른쪽 5", x: -60.19, y: -62.82, theta: 90 },
  { id: "RST6", name: "창고 오른쪽 6", x: -60.19, y: -62.32, theta: 90 },
  { id: "RST7", name: "창고 오른쪽 7", x: -60.19, y: -61.82, theta: 90 },
  { id: "RST8", name: "창고 오른쪽 8", x: -60.19, y: -61.32, theta: 90 },
  { id: "RST9", name: "창고 오른쪽 9", x: -60.19, y: -60.82, theta: 90 },
  { id: "RST10", name: "창고 오른쪽 10", x: -60.19, y: -60.32, theta: 90 },
];

// 위치 유형별 그룹화
const LOCATION_GROUPS = {
  shelves: LOCATIONS.filter((loc) => loc.id.match(/^[A-L][1-2]$/)),
  storage: [
    ...LOCATIONS.filter((loc) => loc.id.startsWith("LST")),
    ...LOCATIONS.filter((loc) => loc.id.startsWith("RST")),
  ],
  other: LOCATIONS.filter((loc) => loc.id.startsWith("EMP")),
};

const PickPlacePage = () => {
  const canvasRef = useRef(null);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [pixelPosition, setPixelPosition] = useState({ pixelX: 0, pixelY: 0 });
  const [mapData, setMapData] = useState(null);
  const { token } = useAuthStore();
  const socketRef = useRef(null);

  // Pick & Place 관련 상태
  const [fromLocation, setFromLocation] = useState("");
  const [toLocation, setToLocation] = useState("");
  const [product, setProduct] = useState(PRODUCTS[0].id);
  const [isLoading, setIsLoading] = useState(false);
  const [commandResult, setCommandResult] = useState(null);

  // 로봇 작업 상태
  const [operationStatus, setOperationStatus] = useState({
    inProgress: false,
    step: null, // 'picking', 'moving', 'placing', 'completed', 'failed'
    message: null,
  });

  // 작업 추적을 위한 상태
  const [currentTask, setCurrentTask] = useState(null);

  // 활성화된 위치 마커 상태
  const [activeLocations, setActiveLocations] = useState({
    source: null,
    destination: null,
  });

  // 맵 데이터 로드
  useEffect(() => {
    if (!token) return;

    const fetchMapData = async () => {
      try {
        const apiUrl = getApiUrl();
        const response = await axios.get(`${apiUrl}/robot/map-processed`, {
          headers: {
            Authorization: `Bearer ${token}`,
          },
        });

        if (response.data && response.data.map) {
          console.log("맵 데이터 로드:", response.data);
          setMapData(response.data);
        }
      } catch (error) {
        console.error("맵 데이터 로드 실패:", error);
      }
    };

    fetchMapData();
  }, [token]);

  // WebSocket 연결 및 이벤트 처리
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
      try {
        const data = JSON.parse(event.data);
        console.log("WebSocket 메시지 수신:", data);

        // 위치 데이터 처리
        if (data.type === "position") {
          const x = data.x;
          const y = data.y;

          setPosition({ x, y });

          // 로봇 좌표를 픽셀 좌표로 변환
          const { pixelX, pixelY } = robotToPixelCoordinates(x, y);
          setPixelPosition({ pixelX, pixelY });
        }
        // 맵 데이터 업데이트 처리
        else if (data.type === "map") {
          console.log("맵 데이터 업데이트 수신");
          setMapData(data);
        }
        // 물건 집기 완료 이벤트
        else if (data.type === "pick_complete") {
          console.log("물건 집기 완료:", data);

          if (data.success) {
            // 현재 작업 정보와 일치하는지 확인
            if (
              currentTask &&
              currentTask.product_id === data.product_id &&
              currentTask.from_id.toLowerCase() === data.from_id.toLowerCase()
            ) {
              setOperationStatus({
                inProgress: true,
                step: "moving",
                message: `✅ ${
                  data.product_id
                } 물품을 ${data.from_id.toUpperCase()}에서 성공적으로 집었습니다. 목적지로 이동 중...`,
              });

              // 출발지를 집음 상태로 표시
              setActiveLocations((prev) => ({
                ...prev,
                source: {
                  id: data.from_id.toUpperCase(),
                  status: "picked",
                },
              }));
            }
          } else {
            setOperationStatus({
              inProgress: false,
              step: "failed",
              message: `❌ ${
                data.product_id
              } 물품을 ${data.from_id.toUpperCase()}에서 집지 못했습니다.`,
            });

            // 실패 후 3초 뒤 상태 초기화
            setTimeout(() => {
              setOperationStatus({
                inProgress: false,
                step: null,
                message: null,
              });
              setCurrentTask(null);
              setActiveLocations({ source: null, destination: null });
            }, 3000);
          }
        }
        // 물건 놓기 완료 이벤트
        else if (typeof data === 'object' && data.type === "place_complete") {
          console.log("물건 놓기 완료:", data);

          if (data.success) {
            // 현재 작업 정보와 일치하는지 확인
            if (
              currentTask &&
              currentTask.product_id === data.product_id &&
              currentTask.to_id.toLowerCase() === data.to_id.toLowerCase()
            ) {
              setOperationStatus({
                inProgress: false,
                step: "completed",
                message: `✅ ${
                  data.product_id
                } 물품을 ${data.to_id.toUpperCase()}에 성공적으로 배치했습니다.`,
              });

              // 목적지를 배치 완료 상태로 표시
              setActiveLocations((prev) => ({
                ...prev,
                destination: {
                  id: data.to_id.toUpperCase(),
                  status: "placed",
                },
              }));

              // 작업 완료 후 3초 뒤 상태 초기화
              setTimeout(() => {
                setOperationStatus({
                  inProgress: false,
                  step: null,
                  message: null,
                });
                setCurrentTask(null);
                setActiveLocations({ source: null, destination: null });
              }, 5000);
            }
          } else {
            setOperationStatus({
              inProgress: false,
              step: "failed",
              message: `❌ ${
                data.product_id
              } 물품을 ${data.to_id.toUpperCase()}에 배치하지 못했습니다.`,
            });

            // 실패 후 3초 뒤 상태 초기화
            setTimeout(() => {
              setOperationStatus({
                inProgress: false,
                step: null,
                message: null,
              });
              setCurrentTask(null);
              setActiveLocations({ source: null, destination: null });
            }, 3000);
          }
        }
        // 맵핑 완료 이벤트 (필요시 활성화)
        else if (data.type === "mapping_complete") {
          console.log("맵핑 완료:", data);
        }
      } catch (error) {
        console.error("WebSocket 데이터 파싱 오류:", error);
      }
    };

    socket.onerror = (error) => {
      console.error("WebSocket 오류:", error);
    };

    socket.onclose = () => {
      console.log("WebSocket 연결 종료");
    };

    return () => {
      if (socket.readyState === WebSocket.OPEN) {
        socket.close();
      }
    };
  }, [token, currentTask]);

  // Canvas에 맵 렌더링
  useEffect(() => {
    if (!mapData || !canvasRef.current) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    const { width, height, map } = mapData;

    // 캔버스 클리어
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 배경을 회색으로 설정 (미탐색 영역)
    ctx.fillStyle = "#e0e0e0";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // 맵 데이터 렌더링 (좌우 반전 적용)
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        // 맵을 좌우로 반전 - x 좌표에서 width-1-x 값을 사용
        const value = map[y][width - 1 - x];

        // 값에 따라 색상 지정
        if (value === 0) {
          // 빈 공간 (탐색 완료, 이동 가능)
          ctx.fillStyle = "#ffffff";
          ctx.fillRect(x, y, 1, 1);
        } else if (value === 100) {
          // 장애물
          ctx.fillStyle = "#333333";
          ctx.fillRect(x, y, 1, 1);
        }
        // -1은 미탐색 영역으로 기본 배경색 사용
      }
    }

    // 모든 위치 마커 그리기
    LOCATIONS.forEach((loc) => {
      const { pixelX, pixelY } = robotToPixelCoordinates(loc.x, loc.y);

      // 위치 유형에 따라 다른 색상 사용
      let markerColor = "#555555";
      let labelColor = "#555555";

      if (loc.id.match(/^[A-L][1-2]$/)) {
        // 랙 위치
        markerColor = "#3498db";
        labelColor = "#3498db";
      } else if (loc.id.startsWith("LST")) {
        // 몽쉘 창고
        markerColor = "#e74c3c";
        labelColor = "#e74c3c";
      } else if (loc.id.startsWith("RST")) {
        // 쿠크다스 창고
        markerColor = "#2ecc71";
        labelColor = "#2ecc71";
      } else if (loc.id.startsWith("EMP")) {
        // 기타 위치
        markerColor = "#f39c12";
        labelColor = "#f39c12";
      }

      // 활성 위치 마커 강조 (출발지/목적지)
      if (activeLocations.source && activeLocations.source.id === loc.id) {
        // 출발지
        markerColor =
          activeLocations.source.status === "picked"
            ? "#d35400" // 집은 상태 (더 어두운 주황색)
            : "#e67e22"; // 선택된 상태 (주황색)
        labelColor = markerColor;
      } else if (
        activeLocations.destination &&
        activeLocations.destination.id === loc.id
      ) {
        // 목적지
        markerColor =
          activeLocations.destination.status === "placed"
            ? "#27ae60" // 배치 완료 (더 어두운 녹색)
            : "#2ecc71"; // 선택된 상태 (녹색)
        labelColor = markerColor;
      }

      // 선택된 출발지/목적지에 강조 표시
      const isFromLocation = fromLocation === loc.id;
      const isToLocation = toLocation === loc.id;

      if (isFromLocation && !activeLocations.source) {
        markerColor = "#e67e22"; // 주황색 (출발지)
        labelColor = markerColor;
      } else if (isToLocation && !activeLocations.destination) {
        markerColor = "#2ecc71"; // 녹색 (목적지)
        labelColor = markerColor;
      }

      // 마커 그리기
      ctx.beginPath();
      ctx.arc(pixelX, pixelY, 4, 0, Math.PI * 2);
      ctx.fillStyle = markerColor;
      ctx.fill();

      // 선택된 위치 또는 활성 위치인 경우 테두리 강조
      if (
        isFromLocation ||
        isToLocation ||
        (activeLocations.source && activeLocations.source.id === loc.id) ||
        (activeLocations.destination &&
          activeLocations.destination.id === loc.id)
      ) {
        ctx.beginPath();
        ctx.arc(pixelX, pixelY, 6, 0, Math.PI * 2);
        ctx.strokeStyle = markerColor;
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // 방향 표시
      const angleRad = (loc.theta * Math.PI) / 180;
      const dirX = pixelX + Math.cos(angleRad) * 8;
      const dirY = pixelY + Math.sin(angleRad) * 8;
      ctx.beginPath();
      ctx.moveTo(pixelX, pixelY);
      ctx.lineTo(dirX, dirY);
      ctx.strokeStyle = markerColor;
      ctx.lineWidth = 2;
      ctx.stroke();

      // ID 라벨
      ctx.font = "9px Arial";
      ctx.fillStyle = labelColor;
      ctx.fillText(loc.id, pixelX + 6, pixelY - 6);
    });

    // 현재 로봇 위치 마커 그리기
    if (pixelPosition.pixelX > 0 && pixelPosition.pixelY > 0) {
      // 외부 원 (흰색 테두리)
      ctx.beginPath();
      ctx.arc(pixelPosition.pixelX, pixelPosition.pixelY, 8, 0, Math.PI * 2);
      ctx.fillStyle = "white";
      ctx.fill();

      // 내부 원 (빨간색)
      ctx.beginPath();
      ctx.arc(pixelPosition.pixelX, pixelPosition.pixelY, 6, 0, Math.PI * 2);
      ctx.fillStyle = "red";
      ctx.fill();

      // 로봇 방향 (추정값 - 실제 방향 데이터가 있으면 수정 필요)
      ctx.beginPath();
      ctx.moveTo(pixelPosition.pixelX, pixelPosition.pixelY);
      ctx.lineTo(pixelPosition.pixelX, pixelPosition.pixelY - 12);
      ctx.strokeStyle = "red";
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }, [mapData, pixelPosition, fromLocation, toLocation, activeLocations]);

  // Pick & Place 명령 전송
  const sendPickPlaceCommand = async () => {
    if (
      !fromLocation ||
      !toLocation ||
      !product ||
      isLoading ||
      operationStatus.inProgress
    )
      return;

    const fromLoc = LOCATIONS.find((loc) => loc.id === fromLocation);
    const toLoc = LOCATIONS.find((loc) => loc.id === toLocation);

    if (!fromLoc || !toLoc) {
      console.error("출발지 또는 목적지를 찾을 수 없습니다.");
      return;
    }

    setIsLoading(true);
    setCommandResult(null);

    // 작업 시작 상태 설정
    setOperationStatus({
      inProgress: true,
      step: "picking",
      message: `🔄 ${product} 물품을 ${fromLocation}에서 ${toLocation}으로 이동하는 중...`,
    });

    // 활성 위치 설정
    setActiveLocations({
      source: { id: fromLocation, status: "selected" },
      destination: { id: toLocation, status: "selected" },
    });

    try {
      const apiUrl = getApiUrl();
      const commandData = {
        from: {
          x: fromLoc.x,
          y: fromLoc.y,
          theta: fromLoc.theta, // 각도(degrees)
        },
        to: {
          x: toLoc.x,
          y: toLoc.y,
          theta: toLoc.theta, // 각도(degrees)
        },
        product_id: product,
        from_id: fromLocation.toUpperCase(),
        to_id: toLocation.toUpperCase(),
      };

      // 현재 작업 정보 저장
      setCurrentTask(commandData);

      console.log("Pick & Place 명령 전송:", commandData);

      const response = await axios.post(
        `${apiUrl}/robot/pick-place`,
        commandData,
        {
          headers: {
            Authorization: `Bearer ${token}`,
          },
        }
      );

      console.log("Pick & Place 명령 응답:", response.data);
      setCommandResult({
        success: true,
        message: `명령이 성공적으로 전송되었습니다: ${product} 물품을 ${fromLocation}에서 ${toLocation}으로 이동`,
        data: response.data,
      });
    } catch (error) {
      console.error("Pick & Place 명령 전송 실패:", error);
      setCommandResult({
        success: false,
        message: `명령 전송에 실패했습니다: ${error.message}`,
        error: error,
      });

      setOperationStatus({
        inProgress: false,
        step: "failed",
        message: `❌ 오류: ${error.message}`,
      });

      // 활성 위치 초기화
      setActiveLocations({ source: null, destination: null });
      setCurrentTask(null);
    } finally {
      setIsLoading(false);
    }
  };

  // 작업 취소
  const cancelOperation = () => {
    // 실제 로봇 작업 취소 API가 있다면 여기서 호출

    setOperationStatus({
      inProgress: false,
      step: null,
      message: null,
    });

    setActiveLocations({ source: null, destination: null });
    setCurrentTask(null);
  };

  // 위치 그룹 필터링 및 정렬
  const getLocationOptions = (groupId) => {
    const locations = LOCATION_GROUPS[groupId] || [];
    return locations.sort((a, b) => a.id.localeCompare(b.id));
  };

  // 작업 진행 상태에 따른 아이콘과 색상
  const getStatusIconAndColor = () => {
    switch (operationStatus.step) {
      case "picking":
        return { icon: "🔄", colorClass: "bg-blue-100 text-blue-800" };
      case "moving":
        return { icon: "🚚", colorClass: "bg-blue-100 text-blue-800" };
      case "placing":
        return { icon: "📦", colorClass: "bg-blue-100 text-blue-800" };
      case "completed":
        return { icon: "✅", colorClass: "bg-green-100 text-green-800" };
      case "failed":
        return { icon: "❌", colorClass: "bg-red-100 text-red-800" };
      default:
        return { icon: "", colorClass: "" };
    }
  };

  const { icon, colorClass } = getStatusIconAndColor();

  return (
    <Layout>
      <div style={{ fontFamily: "Arial", padding: "1rem" }}>
        <h2 className="text-2xl font-bold mb-4">
          🤖 물품 이동 명령 (Pick & Place)
        </h2>

        {/* 상태 메시지 표시 */}
        {operationStatus.message && (
          <div className={`mb-4 p-3 rounded ${colorClass}`}>
            {icon} {operationStatus.message}
            {operationStatus.inProgress && (
              <button
                onClick={cancelOperation}
                className="ml-4 px-2 py-1 bg-gray-200 text-gray-800 text-sm rounded hover:bg-gray-300"
              >
                작업 취소
              </button>
            )}
          </div>
        )}

        <div className="flex flex-wrap">
          {/* 맵 및 명령 영역 */}
          <div className="w-full md:w-8/12 pr-4">
            {/* 맵 캔버스 */}
            <div style={{ position: "relative", marginBottom: "1rem" }}>
              <canvas
                ref={canvasRef}
                width={MAP_WIDTH}
                height={MAP_HEIGHT}
                style={{
                  border: "1px solid #ccc",
                  backgroundColor: "#f0f0f0",
                }}
              />

              {!mapData && (
                <div
                  style={{
                    position: "absolute",
                    top: "50%",
                    left: "50%",
                    transform: "translate(-50%, -50%)",
                    textAlign: "center",
                    color: "#666",
                    pointerEvents: "none",
                  }}
                >
                  맵 데이터 로딩 중...
                </div>
              )}
            </div>

            {/* 좌표 정보 표시 */}
            <p className="mb-2">
              <strong>로봇 좌표:</strong> X: {position.x.toFixed(2)}, Y:{" "}
              {position.y.toFixed(2)}
            </p>

            {/* 선택된 출발지/목적지 표시 */}
            {(fromLocation || toLocation) && (
              <div className="mb-4 p-3 bg-gray-100 rounded">
                <h3 className="font-bold mb-2">선택한 위치:</h3>
                {fromLocation && (
                  <p>
                    <span className="inline-block w-6 h-3 bg-orange-500 mr-2"></span>
                    <strong>출발지:</strong>{" "}
                    {LOCATIONS.find((loc) => loc.id === fromLocation)?.name} (
                    {fromLocation})
                  </p>
                )}
                {toLocation && (
                  <p>
                    <span className="inline-block w-6 h-3 bg-green-500 mr-2"></span>
                    <strong>목적지:</strong>{" "}
                    {LOCATIONS.find((loc) => loc.id === toLocation)?.name} (
                    {toLocation})
                  </p>
                )}
              </div>
            )}

            {/* 명령 결과 표시 */}
            {commandResult && (
              <div
                className={`mt-4 p-3 rounded ${
                  commandResult.success
                    ? "bg-green-100 text-green-800"
                    : "bg-red-100 text-red-800"
                }`}
              >
                <p>
                  <strong>
                    {commandResult.success ? "✅ 성공" : "❌ 실패"}
                  </strong>
                  : {commandResult.message}
                </p>
                {commandResult.data && (
                  <details className="mt-2">
                    <summary className="cursor-pointer font-medium">
                      응답 상세 정보
                    </summary>
                    <pre className="mt-2 text-xs bg-gray-100 p-2 rounded overflow-auto">
                      {JSON.stringify(commandResult.data, null, 2)}
                    </pre>
                  </details>
                )}
              </div>
            )}
          </div>

          {/* 명령 입력 폼 */}
          <div className="w-full md:w-4/12 mt-4 md:mt-0">
            <div className="bg-gray-100 p-4 rounded">
              <h3 className="text-lg font-bold mb-4">📦 물품 이동 명령</h3>

              {/* 물품 선택 */}
              <div className="mb-4">
                <label className="block text-gray-700 mb-2">물품 선택:</label>
                <select
                  value={product}
                  onChange={(e) => setProduct(e.target.value)}
                  className="w-full p-2 border border-gray-300 rounded"
                  disabled={isLoading || operationStatus.inProgress}
                >
                  {PRODUCTS.map((prod) => (
                    <option key={prod.id} value={prod.id}>
                      {prod.name}
                    </option>
                  ))}
                </select>
              </div>

              {/* 출발지 선택 */}
              <div className="mb-4">
                <label className="block text-gray-700 mb-2">
                  출발지 (From):
                </label>

                {/* 출발지 그룹 탭 */}
                <div className="flex mb-2 border-b">
                  <button
                    onClick={() => setFromLocation("")}
                    className={`py-1 px-3 ${
                      !fromLocation
                        ? "border-b-2 border-blue-500 text-blue-500"
                        : "text-gray-500"
                    }`}
                    disabled={isLoading || operationStatus.inProgress}
                  >
                    전체
                  </button>

                  {/* 출발지로 적합한 그룹만 표시 (창고와 기타 위치) */}
                  <button
                    onClick={() =>
                      setFromLocation(LOCATION_GROUPS.storage[0].id)
                    }
                    className={`py-1 px-3 ${
                      fromLocation &&
                      LOCATION_GROUPS.storage.some(
                        (loc) => loc.id === fromLocation
                      )
                        ? "border-b-2 border-blue-500 text-blue-500"
                        : "text-gray-500"
                    }`}
                    disabled={isLoading || operationStatus.inProgress}
                  >
                    창고
                  </button>
                  <button
                    onClick={() =>
                      setFromLocation(LOCATION_GROUPS.shelves[0].id)
                    }
                    className={`py-1 px-3 ${
                      fromLocation &&
                      LOCATION_GROUPS.shelves.some(
                        (loc) => loc.id === fromLocation
                      )
                        ? "border-b-2 border-blue-500 text-blue-500"
                        : "text-gray-500"
                    }`}
                    disabled={isLoading || operationStatus.inProgress}
                  >
                    진열대
                  </button>
                  <button
                    onClick={() => setFromLocation(LOCATION_GROUPS.other[0].id)}
                    className={`py-1 px-3 ${
                      fromLocation &&
                      LOCATION_GROUPS.other.some(
                        (loc) => loc.id === fromLocation
                      )
                        ? "border-b-2 border-blue-500 text-blue-500"
                        : "text-gray-500"
                    }`}
                    disabled={isLoading || operationStatus.inProgress}
                  >
                    기타
                  </button>
                </div>

                {/* 출발지 선택 드롭다운 */}
                <select
                  value={fromLocation}
                  onChange={(e) => setFromLocation(e.target.value)}
                  className="w-full p-2 border border-gray-300 rounded"
                  disabled={isLoading || operationStatus.inProgress}
                >
                  <option value="">출발지 선택</option>

                  {/* 창고 (제품별 자동 필터링) */}
                  <optgroup label="창고 위치">
                    {getLocationOptions("storage")
                      // 제품에 따라 적절한 창고 위치만 표시
                      .filter(
                        (loc) =>
                          (product === "쿠크다스" &&
                            loc.id.startsWith("RST")) ||
                          (product === "몽쉘" && loc.id.startsWith("LST"))
                      )
                      .map((loc) => (
                        <option key={loc.id} value={loc.id}>
                          {loc.name} ({loc.id})
                        </option>
                      ))}
                  </optgroup>

                  {/* 진열대 */}
                  <optgroup label="진열대 위치">
                    {getLocationOptions("shelves").map((loc) => (
                      <option key={loc.id} value={loc.id}>
                        {loc.name} ({loc.id})
                      </option>
                    ))}
                  </optgroup>

                  {/* 기타 위치 */}
                  <optgroup label="기타 위치">
                    {getLocationOptions("other").map((loc) => (
                      <option key={loc.id} value={loc.id}>
                        {loc.name} ({loc.id})
                      </option>
                    ))}
                  </optgroup>
                </select>
              </div>

              {/* 목적지 선택 */}
              <div className="mb-4">
                <label className="block text-gray-700 mb-2">목적지 (To):</label>

                {/* 목적지 그룹 탭 */}
                <div className="flex mb-2 border-b">
                  <button
                    onClick={() => setToLocation("")}
                    className={`py-1 px-3 ${
                      !toLocation
                        ? "border-b-2 border-blue-500 text-blue-500"
                        : "text-gray-500"
                    }`}
                    disabled={isLoading || operationStatus.inProgress}
                  >
                    전체
                  </button>
                  <button
                    onClick={() => setToLocation(LOCATION_GROUPS.shelves[0].id)}
                    className={`py-1 px-3 ${
                      toLocation &&
                      LOCATION_GROUPS.shelves.some(
                        (loc) => loc.id === toLocation
                      )
                        ? "border-b-2 border-blue-500 text-blue-500"
                        : "text-gray-500"
                    }`}
                    disabled={isLoading || operationStatus.inProgress}
                  >
                    진열대
                  </button>
                  <button
                    onClick={() => setToLocation(LOCATION_GROUPS.storage[0].id)}
                    className={`py-1 px-3 ${
                      toLocation &&
                      LOCATION_GROUPS.storage.some(
                        (loc) => loc.id === toLocation
                      )
                        ? "border-b-2 border-blue-500 text-blue-500"
                        : "text-gray-500"
                    }`}
                    disabled={isLoading || operationStatus.inProgress}
                  >
                    창고
                  </button>
                  <button
                    onClick={() => setToLocation(LOCATION_GROUPS.other[0].id)}
                    className={`py-1 px-3 ${
                      toLocation &&
                      LOCATION_GROUPS.other.some((loc) => loc.id === toLocation)
                        ? "border-b-2 border-blue-500 text-blue-500"
                        : "text-gray-500"
                    }`}
                    disabled={isLoading || operationStatus.inProgress}
                  >
                    기타
                  </button>
                </div>

                {/* 목적지 선택 드롭다운 */}
                <select
                  value={toLocation}
                  onChange={(e) => setToLocation(e.target.value)}
                  className="w-full p-2 border border-gray-300 rounded"
                  disabled={isLoading || operationStatus.inProgress}
                >
                  <option value="">목적지 선택</option>

                  {/* 진열대 */}
                  <optgroup label="진열대 위치">
                    {getLocationOptions("shelves").map((loc) => (
                      <option key={loc.id} value={loc.id}>
                        {loc.name} ({loc.id})
                      </option>
                    ))}
                  </optgroup>

                  {/* 창고 (제품별 자동 필터링) */}
                  <optgroup label="창고 위치">
                    {getLocationOptions("storage")
                      // 제품에 따라 적절한 창고 위치만 표시
                      .filter(
                        (loc) =>
                          (product === "쿠크다스" &&
                            loc.id.startsWith("RST")) ||
                          (product === "몽쉘" && loc.id.startsWith("LST"))
                      )
                      .map((loc) => (
                        <option key={loc.id} value={loc.id}>
                          {loc.name} ({loc.id})
                        </option>
                      ))}
                  </optgroup>

                  {/* 기타 위치 */}
                  <optgroup label="기타 위치">
                    {getLocationOptions("other").map((loc) => (
                      <option key={loc.id} value={loc.id}>
                        {loc.name} ({loc.id})
                      </option>
                    ))}
                  </optgroup>
                </select>
              </div>

              {/* 명령 요약 */}
              {fromLocation && toLocation && !operationStatus.inProgress && (
                <div className="mb-4 p-3 bg-blue-50 rounded text-blue-800">
                  <p className="font-medium">🔄 명령 요약:</p>
                  <p>
                    {product} 물품을 {fromLocation} 위치에서 가져와
                  </p>
                  <p>{toLocation} 위치로 이동합니다.</p>
                </div>
              )}

              {/* 명령 전송 버튼 */}
              <button
                onClick={sendPickPlaceCommand}
                disabled={
                  !fromLocation ||
                  !toLocation ||
                  isLoading ||
                  operationStatus.inProgress
                }
                className={`w-full py-2 px-4 rounded font-bold ${
                  !fromLocation ||
                  !toLocation ||
                  isLoading ||
                  operationStatus.inProgress
                    ? "bg-gray-300 text-gray-500 cursor-not-allowed"
                    : "bg-blue-500 text-white hover:bg-blue-600"
                }`}
              >
                {isLoading ? "명령 전송 중..." : "명령 전송"}
              </button>

              {/* 위치 좌표 정보 */}
              <div className="mt-4 border-t pt-4">
                <h4 className="font-bold mb-2">선택한 위치 정보:</h4>

                {fromLocation && (
                  <div className="mb-2">
                    <p className="text-sm">
                      <strong>출발지:</strong>{" "}
                      {LOCATIONS.find((loc) => loc.id === fromLocation)?.name} (
                      {fromLocation})
                    </p>
                    <p className="text-sm text-gray-600">
                      X:{" "}
                      {LOCATIONS.find(
                        (loc) => loc.id === fromLocation
                      )?.x.toFixed(2)}
                      , Y:{" "}
                      {LOCATIONS.find(
                        (loc) => loc.id === fromLocation
                      )?.y.toFixed(2)}
                      , θ:{" "}
                      {LOCATIONS.find((loc) => loc.id === fromLocation)?.theta}°
                    </p>
                  </div>
                )}

                {toLocation && (
                  <div>
                    <p className="text-sm">
                      <strong>목적지:</strong>{" "}
                      {LOCATIONS.find((loc) => loc.id === toLocation)?.name} (
                      {toLocation})
                    </p>
                    <p className="text-sm text-gray-600">
                      X:{" "}
                      {LOCATIONS.find(
                        (loc) => loc.id === toLocation
                      )?.x.toFixed(2)}
                      , Y:{" "}
                      {LOCATIONS.find(
                        (loc) => loc.id === toLocation
                      )?.y.toFixed(2)}
                      , θ:{" "}
                      {LOCATIONS.find((loc) => loc.id === toLocation)?.theta}°
                    </p>
                  </div>
                )}
              </div>
            </div>

            {/* 범례 */}
            <div className="mt-4 bg-gray-100 p-4 rounded">
              <h4 className="font-bold mb-2">📍 맵 범례:</h4>
              <div className="grid grid-cols-2 gap-2 text-sm">
                <div className="flex items-center">
                  <div className="w-3 h-3 bg-red-500 rounded-full mr-2"></div>
                  <span>현재 로봇 위치</span>
                </div>
                <div className="flex items-center">
                  <div className="w-3 h-3 bg-blue-500 rounded-full mr-2"></div>
                  <span>진열대 위치</span>
                </div>
                <div className="flex items-center">
                  <div className="w-3 h-3 bg-green-500 rounded-full mr-2"></div>
                  <span>쿠크다스 창고</span>
                </div>
                <div className="flex items-center">
                  <div className="w-3 h-3 bg-red-500 rounded-full mr-2"></div>
                  <span>몽쉘 창고</span>
                </div>
                <div className="flex items-center">
                  <div className="w-3 h-3 bg-yellow-600 rounded-full mr-2"></div>
                  <span>기타 위치</span>
                </div>
                <div className="flex items-center">
                  <div className="w-3 h-3 bg-orange-500 rounded-full mr-2"></div>
                  <span>선택된 출발지</span>
                </div>
                <div className="flex items-center">
                  <div className="w-3 h-3 bg-green-600 rounded-full mr-2"></div>
                  <span>선택된 목적지</span>
                </div>
                <div className="flex items-center">
                  <div className="w-3 h-3 bg-gray-800 rounded-full mr-2"></div>
                  <span>장애물</span>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* 도움말 */}
        <div className="mt-6 p-4 bg-gray-50 rounded-lg border border-gray-200">
          <h3 className="text-lg font-bold mb-2">📝 사용 방법</h3>
          <ol className="list-decimal ml-5 space-y-2">
            <li>물품 종류(쿠크다스 또는 몽쉘)를 선택합니다.</li>
            <li>
              출발지(From) 위치를 선택합니다. 물품 종류에 따라 적절한 창고
              위치가 필터링됩니다.
            </li>
            <li>목적지(To) 위치를 선택합니다.</li>
            <li>명령 전송 버튼을 클릭하여 로봇에게 명령을 전송합니다.</li>
            <li>
              로봇이 작업을 수행하는 동안 맵에서 현재 위치와 상태를 실시간으로
              확인할 수 있습니다.
            </li>
            <li>
              물건을 집거나 놓을 때 맵 데이터가 실시간으로 업데이트되어 변화를
              확인할 수 있습니다.
            </li>
          </ol>
          <p className="mt-4 text-sm text-gray-600">
            <strong>참고:</strong> 출발지와 목적지의 방향(θ)은 로봇이 해당
            위치에 접근할 때 취해야 할 방향을 나타냅니다. 이 값은 자동으로
            설정되며 수정할 필요가 없습니다.
          </p>
        </div>
      </div>
    </Layout>
  );
};

export default PickPlacePage;
