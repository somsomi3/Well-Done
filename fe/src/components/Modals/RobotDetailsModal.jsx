import React, { useEffect, useState, useRef } from 'react';
import ReactModal from 'react-modal';
import { useAuthStore } from '../../stores/authStore';
import { robotData } from '../../utils/robotData';

ReactModal.setAppElement('#root');

const IMAGE_REFRESH_INTERVAL = 1000;

const RobotDetailsModal = ({ isOpen, onClose, robotId }) => {
  const [imageData, setImageData] = useState(null);
  const [lastUpdated, setLastUpdated] = useState(null);
  const [imageError, setImageError] = useState(false);
  const [loading, setLoading] = useState(true);
  const socketRef = useRef(null);
  const fetchIntervalRef = useRef(null);
  const { token } = useAuthStore();

  const robot = robotData.robots.find((r) => r.id === robotId);
  const isRobot1 = robotId === 1;

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
    const interval = setInterval(fetchCameraImage, IMAGE_REFRESH_INTERVAL);
    return () => clearInterval(interval);
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
        {/* 좌측: 카메라 화면 */}
        <div className="border rounded-lg overflow-hidden">
          {isRobot1 ? (
            loading ? (
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
            )
          ) : (
            <div className="w-full h-full bg-gray-100 flex items-center justify-center">
              <span className="text-gray-500">카메라 미연결</span>
            </div>
          )}
        </div>

        {/* 우측: 활동 기록 */}
        <div className="border rounded-lg overflow-y-auto">
          <table className="w-full text-left border-collapse">
            <thead className="bg-gray-50 sticky top-0 z-10">
              <tr>
                <th className="p-2 border-b">시간</th>
                <th className="p-2 border-b">작업 유형</th>
                <th className="p-2 border-b">상태</th>
                <th className="p-2 border-b">세부 내용</th>
              </tr>
            </thead>
            <tbody>
              {robot.activities.map((activity, i) => (
                <tr key={i} className="border-t hover:bg-gray-50">
                  <td className="p-2 text-sm">
                    {new Date(activity.timestamp).toLocaleDateString('ko-KR', {
                      month: '2-digit',
                      day: '2-digit',
                      hour: '2-digit',
                      minute: '2-digit',
                    })}
                  </td>
                  <td className="p-2 text-sm">
                    <span
                      className={`px-2 py-1 rounded ${
                        activity.type === 'empty_pallet'
                          ? 'bg-blue-100 text-blue-600'
                          : activity.type === 'restock'
                          ? 'bg-green-100 text-green-600'
                          : 'bg-purple-100 text-purple-600'
                      }`}
                    >
                      {activity.type === 'empty_pallet'
                        ? '빈 파렛트 이동'
                        : activity.type === 'restock'
                        ? '재고 보충'
                        : '복합 작업'}
                    </span>
                  </td>
                  <td className="p-2 text-sm">
                    <div className="flex items-center">
                      <div
                        className={`w-3 h-3 rounded-full mr-2 ${
                          activity.status === 'completed'
                            ? 'bg-green-500'
                            : activity.status === 'in_progress'
                            ? 'bg-yellow-500'
                            : 'bg-gray-300'
                        }`}
                      />
                      {activity.status === 'completed'
                        ? '완료'
                        : activity.status === 'in_progress'
                        ? '진행 중'
                        : '대기 중'}
                    </div>
                  </td>
                  <td className="p-2 text-sm">
                    {activity.notes}
                    <br />
                    <span className="text-gray-500 text-xs">
                      {activity.start_location} → {activity.end_location}
                    </span>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </ReactModal>
  );
};

export default RobotDetailsModal;
