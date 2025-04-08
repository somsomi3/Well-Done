import React, { useEffect, useState, useRef } from 'react';
import ReactModal from 'react-modal';
import { Tab } from '@headlessui/react';
import { useAuthStore } from '../../stores/authStore';
import { robotData } from '../../utils/robotData';

ReactModal.setAppElement('#root');

const IMAGE_REFRESH_INTERVAL = 1000;

const RobotDetailsModal = ({ isOpen, onClose, robotId }) => {
  const [imageData, setImageData] = useState(null);
  const [imageError, setImageError] = useState(false);
  const [loading, setLoading] = useState(true);
  const socketRef = useRef(null);
  const { token } = useAuthStore();

  const robot = robotData.robots.find((r) => r.id === robotId);
  const isRobot1 = robotId === 1;

  // 진행 중 작업
  const currentTask = robot.activities.find((a) => a.status === 'in_progress');

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

  return (
    <ReactModal
      isOpen={isOpen}
      onRequestClose={onClose}
      contentLabel={`Robot #${robotId} 상세 정보`}
      className="w-[90vw] h-[80vh] max-w-7xl bg-white rounded-lg flex flex-col"
      overlayClassName="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center"
    >
      {/* 헤더 */}
      <div className="flex justify-between items-center p-4 border-b">
        <h2 className="text-2xl font-bold">Robot #{robotId} 상세 정보</h2>
        <button
          onClick={onClose}
          className="text-3xl hover:text-red-500 transition-colors"
        >
          &times;
        </button>
      </div>

      {/* 본문 */}
      <div className="flex-1 grid grid-cols-2 gap-4 p-4 overflow-hidden">
        {/* 좌측: 카메라 화면 및 현재 작업 */}
        <div className="border rounded-lg overflow-hidden flex flex-col">
          {/* 카메라 화면 */}
          <div className="flex-grow bg-gray-100 border-b">
            {loading ? (
              <div className="text-center text-gray-600">로딩 중...</div>
            ) : imageError ? (
              <div className="text-center text-red-500">연결 오류</div>
            ) : imageData ? (
              <img
                src={`data:image/jpeg;base64,${imageData}`}
                className="w-full h-full object-contain"
                alt={`Robot ${robotId} 카메라 화면`}
              />
            ) : (
              <div className="text-center text-gray-600">데이터 없음</div>
            )}
          </div>

          {/* 현재 작업 */}
          <div className="p-4 bg-gray-50">
            <h3 className="text-lg font-semibold mb-2">현재 작업:</h3>
            {currentTask ? (
              <div>
                <p>
                  <strong>작업 유형:</strong> {currentTask.type === 'empty_pallet'
                    ? '빈 파렛트 이동'
                    : currentTask.type === 'restock'
                    ? '재고 보충'
                    : '복합 작업'}
                </p>
                <p>
                  <strong>출발지:</strong> {currentTask.start_location}
                </p>
                <p>
                  <strong>목적지:</strong> {currentTask.end_location}
                </p>
                <p>
                  <strong>상세 내용:</strong> {currentTask.notes}
                </p>
              </div>
            ) : (
              <p className="text-gray-500">현재 진행 중인 작업이 없습니다.</p>
            )}
          </div>
        </div>

        {/* 우측: 해야 할 일 / 완료한 일 */}
        <div className="border rounded-lg overflow-y-auto">
          <Tab.Group>
            {/* 탭 목록 */}
            <Tab.List className="flex space-x-1 bg-gray-50 p-1">
              {['해야 할 일', '완료한 일'].map((category) => (
                <Tab
                  key={category}
                  className={({ selected }) =>
                    `w-full py-2 text-sm font-medium rounded transition-colors ${
                      selected
                        ? 'bg-blue-500 text-white'
                        : 'text-gray-600 hover:bg-gray-100'
                    }`
                  }
                >
                  {category}
                </Tab>
              ))}
            </Tab.List>

            {/* 탭 내용 */}
            <Tab.Panels className="mt-2">
              {/* 해야 할 일 */}
              <Tab.Panel>
                {robot.activities
                  .filter((a) => a.status === 'pending')
                  .map((activity, i) => (
                    <div key={i} className="border-b p-2">
                      <p><strong>작업 유형:</strong> {activity.type}</p>
                      <p><strong>출발지:</strong> {activity.start_location}</p>
                      <p><strong>목적지:</strong> {activity.end_location}</p>
                    </div>
                  ))}
              </Tab.Panel>

              {/* 완료한 일 */}
              <Tab.Panel>
                {robot.activities
                  .filter((a) => a.status === 'completed')
                  .map((activity, i) => (
                    <div key={i} className="border-b p-2">
                      <p><strong>작업 유형:</strong> {activity.type}</p>
                      <p><strong>출발지:</strong> {activity.start_location}</p>
                      <p><strong>목적지:</strong> {activity.end_location}</p>
                      <p><strong>상세 내용:</strong> {activity.notes}</p>
                    </div>
                  ))}
              </Tab.Panel>
            </Tab.Panels>
          </Tab.Group>
        </div>
      </div>
    </ReactModal>
  );
};

export default RobotDetailsModal;
