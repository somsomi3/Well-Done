import React from 'react';

/**
 * 로봇 카드 컴포넌트
 * 
 * @param {Object} props
 * @param {number} props.id - 로봇 ID
 * @param {string} props.status - 로봇 상태 ('active', 'standby', 'charging', 'error')
 * @param {number} props.battery - 배터리 잔량 (0-100)
 * @param {string} props.location - 현재 위치
 * @param {string} props.lastActivity - 마지막 활동 시간
 * @param {Function} props.onDetailClick - 상세 정보 버튼 클릭 핸들러
 */
const RobotCard = ({ 
  id, 
  status = 'standby', 
  battery = 0, 
  location = '알 수 없음', 
  lastActivity = '없음',
  onDetailClick 
}) => {
  // 상태에 따른 색상 및 텍스트 설정
  const statusConfig = {
    active: { color: 'bg-green-500', text: '활성 상태' },
    standby: { color: 'bg-yellow-500', text: '대기 중' },
    charging: { color: 'bg-blue-500', text: '충전 중' },
    error: { color: 'bg-red-500', text: '오류' }
  };

  const { color, text } = statusConfig[status] || statusConfig.standby;
  
  // 배터리 잔량에 따른 색상 설정
  const getBatteryColor = () => {
    if (battery >= 70) return 'text-green-600';
    if (battery >= 30) return 'text-yellow-600';
    return 'text-red-600';
  };

  return (
    <div className="bg-white rounded-xl shadow-lg border border-gray-100 transition-all duration-200 hover:shadow-xl">
      <div className="p-5">
        {/* 헤더 영역 */}
        <div className="flex justify-between items-center mb-4">
          <h2 className="text-xl font-bold text-gray-800">로봇 #{id}</h2>
          <div className="flex items-center">
            <div className={`w-3 h-3 rounded-full ${color} mr-2`}></div>
            <span className="text-sm font-medium text-gray-600">{text}</span>
          </div>
        </div>
        
        {/* 정보 영역 */}
        <div className="space-y-3 mb-5">
          <div className="flex items-center">
            <div className="w-8 text-gray-400">
              <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" strokeWidth={1.5} stroke="currentColor" className="w-5 h-5">
                <path strokeLinecap="round" strokeLinejoin="round" d="M21 10.5h.375c.621 0 1.125.504 1.125 1.125v2.25c0 .621-.504 1.125-1.125 1.125H21M4.5 10.5h6.75V15H4.5v-4.5ZM3.75 18h15A2.25 2.25 0 0 0 21 15.75v-6a2.25 2.25 0 0 0-2.25-2.25h-15A2.25 2.25 0 0 0 1.5 9.75v6A2.25 2.25 0 0 0 3.75 18Z" />
              </svg>
            </div>
            <div className={`ml-2 font-medium ${getBatteryColor()}`}>
              배터리: {battery}%
            </div>
          </div>
          
          <div className="flex items-center">
            <div className="w-8 text-gray-400">
              <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" strokeWidth={1.5} stroke="currentColor" className="w-5 h-5">
                <path strokeLinecap="round" strokeLinejoin="round" d="M15 10.5a3 3 0 1 1-6 0 3 3 0 0 1 6 0Z" />
                <path strokeLinecap="round" strokeLinejoin="round" d="M19.5 10.5c0 7.142-7.5 11.25-7.5 11.25S4.5 17.642 4.5 10.5a7.5 7.5 0 1 1 15 0Z" />
              </svg>
            </div>
            <div className="ml-2 font-medium text-gray-700">
              위치: {location}
            </div>
          </div>
          
          <div className="flex items-center">
            <div className="w-8 text-gray-400">
              <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" strokeWidth={1.5} stroke="currentColor" className="w-5 h-5">
                <path strokeLinecap="round" strokeLinejoin="round" d="M12 6v6h4.5m4.5 0a9 9 0 1 1-18 0 9 9 0 0 1 18 0Z" />
              </svg>
            </div>
            <div className="ml-2 font-medium text-gray-700">
              마지막 활동: {lastActivity}
            </div>
          </div>
        </div>
        
        {/* 버튼 영역 */}
        <div className="flex justify-end">
          <button 
            onClick={() => onDetailClick(id)}
            className="px-4 py-2 bg-blue-600 text-white rounded-lg font-medium transition-colors hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2"
          >
            상세 정보
          </button>
        </div>
      </div>
    </div>
  );
};

export default RobotCard;
