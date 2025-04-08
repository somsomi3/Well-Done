import React from 'react';

const MapControls = ({
  isAutoMapping,
  isLoading,
  isMappingComplete,
  useInflatedMap,
  startAutoMapping,
  stopAutoMapping,
  toggleMapType
}) => {
  return (
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
  );
};

export default MapControls;
