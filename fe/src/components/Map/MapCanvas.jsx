import React, { useEffect, useRef } from 'react';
import { robotToPixelCoordinates } from '../../utils/mapUtils';

const MapCanvas = ({ 
  mapData, 
  finalMapData, 
  inflatedMapData, 
  useInflatedMap, 
  isMappingComplete, 
  pixelPosition, 
  path, 
  onMapClick,
  style // 스타일 prop 추가
}) => {
  const canvasRef = useRef(null);

  useEffect(() => {
    // 매핑이 완료되었다면 finalMapData를 사용, 아니면 실시간 mapData 사용
    const currentMapData = useInflatedMap && inflatedMapData 
                       ? inflatedMapData 
                       : isMappingComplete ? finalMapData : mapData;
    
    if (!currentMapData || !canvasRef.current) return;

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
  }, [mapData, finalMapData, inflatedMapData, useInflatedMap, isMappingComplete, pixelPosition, path]);

  // 기본 스타일과 전달받은 스타일을 병합
  const defaultStyle = {
    border: "1px solid #ccc",
    backgroundColor: "#f0f0f0",
    cursor: "crosshair",
    width: "100%",
    height: "auto",
    maxHeight: "70vh",
    objectFit: "contain"
  };

  // 스타일 병합 (전달받은 스타일이 우선)
  const canvasStyle = { ...defaultStyle, ...(style || {}) };

  return (
    <canvas
      ref={canvasRef}
      width={480}
      height={480}
      style={canvasStyle}
      onClick={onMapClick}
    />
  );
};

export default MapCanvas;
