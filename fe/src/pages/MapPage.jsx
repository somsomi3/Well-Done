import { getApiUrl } from "../configs/env";
import React, { useEffect, useRef, useState } from "react";
import Layout from "../components/Layout/Layout";
import { useAuthStore } from "../stores/authStore";
import axios from "axios";
import MapCanvas from "../components/Map/MapCanvas";
import MapControls from "../components/Map/MapControls";
import MapInfo from "../components/Map/MapInfo";
import {
  robotToPixelCoordinates,
  pixelToRobotCoordinates,
} from "../utils/mapUtils";

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
            Authorization: `Bearer ${token}`,
          },
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
      try {
        const data = JSON.parse(event.data);

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
          setPath((prev) => {
            // 경로가 비어있거나, 새 위치가 이전 위치와 다른 경우에만 추가
            if (
              prev.length === 0 ||
              prev[prev.length - 1].x !== data.x ||
              prev[prev.length - 1].y !== data.y
            ) {
              return [...prev, { x: data.x, y: data.y }];
            }
            return prev; // 동일한 위치면 경로 유지
          });

          // 로봇 좌표를 픽셀 좌표로 변환 (변환 함수 사용)
          const { pixelX, pixelY } = robotToPixelCoordinates(data.x, data.y);
          console.log("변환된 픽셀 좌표:", pixelX, pixelY);
          setPixelPosition({ pixelX, pixelY });
        } else if (data.x !== undefined && data.y !== undefined && !data.type) {
          console.log("📍 기존 형식 위치 데이터 수신:", data.x, data.y);
          setPosition(data);
          setPath((prev) => {
            // 경로가 비어있거나, 새 위치가 이전 위치와 다른 경우에만 추가
            if (
              prev.length === 0 ||
              prev[prev.length - 1].x !== data.x ||
              prev[prev.length - 1].y !== data.y
            ) {
              return [...prev, data];
            }
            return prev; // 동일한 위치면 경로 유지
          });

          const { pixelX, pixelY } = robotToPixelCoordinates(data.x, data.y);
          console.log("변환된 픽셀 좌표:", pixelX, pixelY);
          setPixelPosition({ pixelX, pixelY });
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
          Authorization: `Bearer ${token}`,
        },
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
      const apiUrl = getApiUrl();
      const response = await axios.get(`${apiUrl}/robot/map-inflated`, {
        headers: {
          Authorization: `Bearer ${token}`,
        },
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
      인플레이티드데이터있음: !!inflatedMapData,
    });

    // 인플레이티드 맵으로 전환하려는데 데이터가 없는 경우
    if (!useInflatedMap && !inflatedMapData) {
      console.log(
        "🔍 인플레이티드 맵 데이터가 없습니다. 서버에서 가져오는 중..."
      );
      const success = await fetchInflatedMap();
      console.log(`가져오기 결과: ${success ? "성공" : "실패"}`);
      if (!success) {
        console.log("❌ 가져오기 실패로 맵 전환 취소");
        return;
      }
    }

    // 맵 타입 전환
    setUseInflatedMap((prev) => {
      console.log(
        `🔄 맵 타입 전환: ${prev ? "인플레이티드 맵" : "기본 맵"}에서 ${
          !prev ? "인플레이티드 맵" : "기본 맵"
        }으로`
      );
      return !prev;
    });
  };

  // 오토 맵핑 시작 함수
  const startAutoMapping = async () => {
    if (isAutoMapping || isLoading) return;

    setIsLoading(true);
    try {
      const apiUrl = getApiUrl();
      const url = `${apiUrl}/robot/auto-map`;

      console.log("API 요청 URL:", url);

      const response = await axios.post(
        url,
        { data: true },
        {
          headers: {
            Authorization: `Bearer ${token}`,
          },
        }
      );

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
      const apiUrl = getApiUrl();
      const response = await axios.post(
        `${apiUrl}/robot/stop-auto-map`,
        { data: true },
        {
          headers: {
            Authorization: `Bearer ${token}`,
          },
        }
      );

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

    console.log(
      "맵 클릭 - 픽셀:",
      pixelX,
      pixelY,
      "로봇:",
      robotX.toFixed(2),
      robotY.toFixed(2)
    );

    // 위치 및 경로 업데이트
    const newPosition = { x: robotX, y: robotY };
    setPosition(newPosition);
    setPath((prev) => {
      if (
        prev.length === 0 ||
        prev[prev.length - 1].x !== robotX ||
        prev[prev.length - 1].y !== robotY
      ) {
        return [...prev, newPosition];
      }
      return prev; // 동일한 위치면 경로 유지
    });
    setPixelPosition({ pixelX, pixelY });
  };

  return (
    <Layout>
      <div className="p-4">
        <h2 className="text-2xl font-bold mb-4">📍 실시간 로봇 위치 보기</h2>

        <MapControls
          isAutoMapping={isAutoMapping}
          isLoading={isLoading}
          isMappingComplete={isMappingComplete}
          useInflatedMap={useInflatedMap}
          startAutoMapping={startAutoMapping}
          stopAutoMapping={stopAutoMapping}
          toggleMapType={toggleMapType}
        />

        {/* 맵핑 및 뷰 상태 표시 */}
        {isAutoMapping && (
          <div className="mb-4 p-2 bg-blue-100 text-blue-800 rounded">
            🔄 오토 맵핑이 진행 중입니다...
          </div>
        )}

        {isMappingComplete && (
          <div className="mb-4 p-2 bg-green-100 text-green-800 rounded">
            ✅ 맵핑이 완료되었습니다!{" "}
            {useInflatedMap ? "인플레이티드 맵" : "기본 맵"}을 사용 중입니다.
          </div>
        )}

        <div className="relative mb-4">
          <MapCanvas
            mapData={mapData}
            finalMapData={finalMapData}
            inflatedMapData={inflatedMapData}
            useInflatedMap={useInflatedMap}
            isMappingComplete={isMappingComplete}
            pixelPosition={pixelPosition}
            path={path}
            onMapClick={handleMapClick}
          />

          {!mapData && !finalMapData && (
            <div className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 text-center text-gray-500">
              맵 데이터 로딩 중...
            </div>
          )}
        </div>

        <MapInfo
          position={position}
          pixelPosition={pixelPosition}
          mapData={useInflatedMap ? inflatedMapData : finalMapData || mapData}
          useInflatedMap={useInflatedMap}
        />

        {/* 경로 기록 */}
        <div className="mt-4">
          <h3 className="text-lg font-semibold">🛤 이동 경로</h3>
          <div className="max-h-[150px] overflow-y-auto bg-gray-50 p-2 rounded">
            {path.length > 0 ? (
              <ul className="list-disc pl-5">
                {path.map((point, index) => (
                  <li key={index} className="text-sm">
                    위치 {index + 1}: X={point.x.toFixed(2)}, Y=
                    {point.y.toFixed(2)}
                  </li>
                ))}
              </ul>
            ) : (
              <p className="text-gray-500">경로 기록이 없습니다.</p>
            )}
          </div>
          <p className="text-xs text-gray-500 mt-1">
            총 {path.length}개의 경로 포인트가 기록되었습니다.
          </p>
        </div>
      </div>
    </Layout>
  );
};

export default MapPage;
