import React, { useEffect, useState, useRef } from 'react';
import ReactModal from 'react-modal';
import { useAuthStore } from '../../stores/authStore';
import { robotData } from '../../utils/robotData';

// ReactModal 접근성 설정
ReactModal.setAppElement('#root');

const RobotDetailsModal = ({ isOpen, onClose, robotId }) => {
  // 상태 관리
  const [imageData, setImageData] = useState(null);
  const [lastUpdated, setLastUpdated] = useState(null);
  const [imageError, setImageError] = useState(false);
  const [loading, setLoading] = useState(true);
  const [activeTab, setActiveTab] = useState('pending');
  
  // 참조 관리
  const socketRef = useRef(null);
  const { token } = useAuthStore();
  
  // 로봇 데이터 찾기
  const robot = robotData.robots.find((r) => r.id === robotId);
  const isRobot1 = robotId === 1;
  
  // 현재 진행 중인 작업 찾기
  const currentTask = robot?.activities.find((a) => a.status === 'in_progress');

  // WebSocket 연결 관리 (로봇 1만)
  useEffect(() => {
    if (!isOpen || !isRobot1 || !token) return;

    const socket = new WebSocket(
      `wss://j12e102.p.ssafy.io/ws/user?token=${token}`
    );
    socketRef.current = socket;

    const handleMessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.type === 'camera_image' && data.data) {
          setImageData(data.data);
          setLastUpdated(new Date());
          setLoading(false);
          setImageError(false);
        }
      } catch (error) {
        console.error('데이터 파싱 오류:', error);
        setImageError(true);
      }
    };

    socket.addEventListener('open', () => {
      console.log('WebSocket 연결됨');
      socket.send(JSON.stringify({ type: 'join' }));
    });

    socket.addEventListener('message', handleMessage);

    return () => {
      if (socket.readyState === WebSocket.OPEN) {
        socket.close();
        console.log('❌ WebSocket 안전하게 종료');
      }
      socket.removeEventListener('message', handleMessage);
    };
  }, [isOpen, isRobot1, token]);

  // HTTP 폴링 관리 (로봇 1만)
  useEffect(() => {
    if (!isOpen || !isRobot1 || !token) return;

    const fetchCameraImage = async () => {
      try {
        const response = await fetch(
          'https://j12e102.p.ssafy.io/api/robot/image-jpeg-compressed',
          { headers: { Authorization: `Bearer ${token}` } }
        );

        if (!response.ok) throw new Error(`HTTP 오류: ${response.status}`);
        const data = await response.json();

        if (data?.data) {
          setImageData(data.data);
          setLastUpdated(new Date());
          setLoading(false);
          setImageError(false);
        }
      } catch (error) {
        console.error('이미지 가져오기 오류:', error);
        setImageError(true);
      }
    };

    fetchCameraImage();
    const interval = setInterval(fetchCameraImage, 1000);
    return () => clearInterval(interval);
  }, [isOpen, isRobot1, token]);

  // 작업 유형에 따른 스타일 및 텍스트 반환
  const getTaskTypeStyle = (type) => {
    switch (type) {
      case 'empty_pallet':
        return {
          bgColor: 'bg-blue-100',
          textColor: 'text-blue-800',
          label: '빈 파렛트 이동'
        };
      case 'restock':
        return {
          bgColor: 'bg-green-100',
          textColor: 'text-green-800',
          label: '재고 보충'
        };
      case 'combined':
        return {
          bgColor: 'bg-purple-100',
          textColor: 'text-purple-800',
          label: '복합 작업'
        };
      default:
        return {
          bgColor: 'bg-gray-100',
          textColor: 'text-gray-800',
          label: '기타 작업'
        };
    }
  };

  // 작업 상태에 따른 스타일 및 텍스트 반환
  const getTaskStatusStyle = (status) => {
    switch (status) {
      case 'completed':
        return {
          dotColor: 'bg-green-500',
          label: '완료'
        };
      case 'in_progress':
        return {
          dotColor: 'bg-yellow-500',
          label: '진행 중'
        };
      case 'pending':
        return {
          dotColor: 'bg-gray-300',
          label: '대기 중'
        };
      default:
        return {
          dotColor: 'bg-gray-300',
          label: '알 수 없음'
        };
    }
  };

  // 날짜 포맷팅 함수
  const formatDate = (dateString) => {
    const date = new Date(dateString);
    return new Intl.DateTimeFormat('ko-KR', {
      month: '2-digit',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit'
    }).format(date);
  };

  if (!robot) {
    return null;
  }

  return (
    <ReactModal
      isOpen={isOpen}
      onRequestClose={onClose}
      contentLabel={`Robot #${robotId} 상세 정보`}
      className="w-[90vw] h-[85vh] max-w-7xl bg-white rounded-xl shadow-2xl overflow-hidden border border-gray-200"
      overlayClassName="fixed inset-0 bg-black/50 backdrop-blur-sm flex items-center justify-center z-50"
      shouldCloseOnOverlayClick={true}
      shouldCloseOnEsc={true}
    >
      {/* 헤더 */}
      <div className="flex justify-between items-center p-5 border-b border-gray-200 bg-gray-50">
        <h2 className="text-2xl font-bold text-gray-800">
          <span className="text-blue-600">Robot #{robotId}</span> 상세 정보
        </h2>
        <button
          onClick={onClose}
          className="text-gray-500 hover:text-gray-700 focus:outline-none focus:ring-2 focus:ring-blue-500 rounded-full p-1"
          aria-label="닫기"
        >
          <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
          </svg>
        </button>
      </div>

      {/* 본문 */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 p-6 h-[calc(100%-4rem)] overflow-hidden">
        {/* 좌측: 카메라 화면 및 현재 작업 */}
        <div className="flex flex-col h-full overflow-hidden">
          {/* 카메라 화면 */}
          <div className="relative bg-gray-900 rounded-lg overflow-hidden flex-grow mb-4">
            {isRobot1 ? (
              loading ? (
                <div className="absolute inset-0 flex items-center justify-center">
                  <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
                </div>
              ) : imageError ? (
                <div className="absolute inset-0 flex items-center justify-center text-red-500">
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-12 w-12 mb-2" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
                  </svg>
                  <span className="text-lg font-medium">연결 오류</span>
                </div>
              ) : imageData ? (
                <img
                  src={`data:image/jpeg;base64,${imageData}`}
                  className="w-full h-full object-contain"
                  alt={`Robot ${robotId} 카메라 화면`}
                />
              ) : (
                <div className="absolute inset-0 flex items-center justify-center text-gray-400">
                  <span>데이터 없음</span>
                </div>
              )
            ) : (
              <div className="absolute inset-0 flex flex-col items-center justify-center text-gray-400">
                <svg xmlns="http://www.w3.org/2000/svg" className="h-12 w-12 mb-2" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M3 9a2 2 0 012-2h.93a2 2 0 001.664-.89l.812-1.22A2 2 0 0110.07 4h3.86a2 2 0 011.664.89l.812 1.22A2 2 0 0018.07 7H19a2 2 0 012 2v9a2 2 0 01-2 2H5a2 2 0 01-2-2V9z" />
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 13a3 3 0 11-6 0 3 3 0 016 0z" />
                </svg>
                <span className="text-lg font-medium">카메라 미연결</span>
              </div>
            )}
            
            {/* 카메라 상태 표시 */}
            {isRobot1 && lastUpdated && (
              <div className="absolute bottom-2 right-2 bg-black/50 text-white text-xs px-2 py-1 rounded">
                마지막 업데이트: {lastUpdated.toLocaleTimeString()}
              </div>
            )}
          </div>

          {/* 현재 작업 */}
          <div className="bg-white rounded-lg border border-gray-200 p-4 shadow-sm">
            <h3 className="text-lg font-semibold text-gray-800 mb-3 flex items-center">
              <svg xmlns="http://www.w3.org/2000/svg" className="h-5 w-5 mr-2 text-blue-500" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2" />
              </svg>
              현재 작업
            </h3>
            
            {currentTask ? (
              <div className="space-y-3">
                <div className="flex items-center">
                  <span className="font-medium text-gray-700 w-20">작업 유형:</span>
                  <span className={`px-2 py-1 rounded-full text-sm ${getTaskTypeStyle(currentTask.type).bgColor} ${getTaskTypeStyle(currentTask.type).textColor}`}>
                    {getTaskTypeStyle(currentTask.type).label}
                  </span>
                </div>
                
                <div className="flex items-start">
                  <span className="font-medium text-gray-700 w-20">경로:</span>
                  <span className="text-gray-600">
                    {currentTask.start_location} → {currentTask.end_location}
                  </span>
                </div>
                
                <div className="flex items-start">
                  <span className="font-medium text-gray-700 w-20">상세 내용:</span>
                  <span className="text-gray-600">{currentTask.notes}</span>
                </div>
                
                <div className="flex items-center">
                  <span className="font-medium text-gray-700 w-20">시작 시간:</span>
                  <span className="text-gray-600">{formatDate(currentTask.timestamp)}</span>
                </div>
              </div>
            ) : (
              <div className="flex items-center justify-center h-20 text-gray-500">
                현재 진행 중인 작업이 없습니다.
              </div>
            )}
          </div>
        </div>

        {/* 우측: 작업 목록 탭 */}
        <div className="bg-white rounded-lg border border-gray-200 shadow-sm overflow-hidden flex flex-col h-full">
          {/* 탭 버튼 */}
          <div className="flex border-b border-gray-200">
            <button
              onClick={() => setActiveTab('pending')}
              className={`flex-1 py-3 px-4 text-center font-medium transition-colors ${
                activeTab === 'pending'
                  ? 'bg-blue-50 text-blue-600 border-b-2 border-blue-600'
                  : 'text-gray-600 hover:bg-gray-50'
              }`}
            >
              해야 할 일
            </button>
            <button
              onClick={() => setActiveTab('completed')}
              className={`flex-1 py-3 px-4 text-center font-medium transition-colors ${
                activeTab === 'completed'
                  ? 'bg-blue-50 text-blue-600 border-b-2 border-blue-600'
                  : 'text-gray-600 hover:bg-gray-50'
              }`}
            >
              완료한 일
            </button>
          </div>

          {/* 탭 내용 */}
          <div className="flex-grow overflow-y-auto p-4">
            {activeTab === 'pending' && (
              <div className="space-y-4">
                <h3 className="text-lg font-semibold text-gray-800">대기 중인 작업</h3>
                {robot.activities
                  .filter((a) => a.status === 'pending')
                  .map((activity, index) => (
                    <div key={index} className="border border-gray-200 rounded-lg p-4 hover:bg-gray-50 transition-colors">
                      <div className="flex justify-between items-start mb-2">
                        <span className={`px-2 py-1 rounded-full text-sm ${getTaskTypeStyle(activity.type).bgColor} ${getTaskTypeStyle(activity.type).textColor}`}>
                          {getTaskTypeStyle(activity.type).label}
                        </span>
                        <span className="text-sm text-gray-500">{formatDate(activity.timestamp)}</span>
                      </div>
                      <div className="mb-2 font-medium">{activity.notes}</div>
                      <div className="text-sm text-gray-600">
                        {activity.start_location} → {activity.end_location}
                      </div>
                    </div>
                  ))}
                {robot.activities.filter((a) => a.status === 'pending').length === 0 && (
                  <div className="text-center py-8 text-gray-500">
                    대기 중인 작업이 없습니다.
                  </div>
                )}
              </div>
            )}

            {activeTab === 'completed' && (
              <div className="space-y-4">
                <h3 className="text-lg font-semibold text-gray-800">완료된 작업</h3>
                {robot.activities
                  .filter((a) => a.status === 'completed')
                  .map((activity, index) => (
                    <div key={index} className="border border-gray-200 rounded-lg p-4 hover:bg-gray-50 transition-colors">
                      <div className="flex justify-between items-start mb-2">
                        <span className={`px-2 py-1 rounded-full text-sm ${getTaskTypeStyle(activity.type).bgColor} ${getTaskTypeStyle(activity.type).textColor}`}>
                          {getTaskTypeStyle(activity.type).label}
                        </span>
                        <span className="text-sm text-gray-500">{formatDate(activity.timestamp)}</span>
                      </div>
                      <div className="mb-2 font-medium">{activity.notes}</div>
                      <div className="flex justify-between items-center">
                        <div className="text-sm text-gray-600">
                          {activity.start_location} → {activity.end_location}
                        </div>
                        <div className="flex items-center">
                          <div className={`w-2 h-2 rounded-full ${getTaskStatusStyle(activity.status).dotColor} mr-1`}></div>
                          <span className="text-xs text-gray-500">{getTaskStatusStyle(activity.status).label}</span>
                        </div>
                      </div>
                    </div>
                  ))}
                {robot.activities.filter((a) => a.status === 'completed').length === 0 && (
                  <div className="text-center py-8 text-gray-500">
                    완료된 작업이 없습니다.
                  </div>
                )}
              </div>
            )}
          </div>
        </div>
      </div>
    </ReactModal>
  );
};

export default RobotDetailsModal;
