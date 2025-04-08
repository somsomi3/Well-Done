import React from 'react';

const MapInfo = ({ position, pixelPosition, mapData, useInflatedMap }) => {
  return (
    <div className="space-y-2">
      <p><strong>로봇 좌표:</strong> X: {position.x.toFixed(2)}, Y: {position.y.toFixed(2)}</p>
      <p><strong>픽셀 좌표:</strong> X: {pixelPosition.pixelX}, Y: {pixelPosition.pixelY}</p>
      
      {mapData && (
        <div>
          <h3 className="text-lg font-semibold">🗺️ 맵 정보</h3>
          <p>크기: {mapData.width}x{mapData.height} 픽셀</p>
          <p>해상도: {mapData.resolution}m/px</p>
          <p>적용된 변환: 맵 좌우 반전, 좌표 180도 회전</p>
          {useInflatedMap && (
            <p><strong>현재 보기:</strong> 인플레이티드 맵 (장애물 주변에 안전 마진 추가됨)</p>
          )}
        </div>
      )}
      
      {/* 경로 기록 */}
      <h3 className="text-lg font-semibold">🛤 이동 경로</h3>
      <div className="max-h-[150px] overflow-y-auto bg-gray-50 p-2 w-[400px]">
        {/* 경로 표시 내용은 MapPage에서 처리 */}
      </div>
    </div>
  );
};

export default MapInfo;
