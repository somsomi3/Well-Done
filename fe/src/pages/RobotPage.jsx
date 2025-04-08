import React, { useState } from 'react';
import Layout from '../components/Layout/Layout';
import RobotDetailsModal from '../components/Modals/RobotDetailsModal';

function RobotPage() {
  const [selectedRobot, setSelectedRobot] = useState(null);

  const handleDetailClick = (robotId) => {
    setSelectedRobot(robotId);
  };

  const handleCloseModal = () => {
    setSelectedRobot(null);
  };

  return (
    <Layout>
      <div className="p-6">
        <h1 className="text-2xl font-bold mb-4">로봇 관리 페이지</h1>

        {/* 로봇 카드 목록 */}
        <div className="mt-6 grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          {/* 로봇 #1 카드 */}
          <div className="bg-white p-4 rounded-lg shadow">
            <h2 className="text-lg font-semibold">로봇 #1</h2>
            <div className="flex items-center mt-2">
              <div className="w-3 h-3 rounded-full bg-green-500 mr-2"></div>
              <span>활성 상태</span>
            </div>
            <div className="mt-3">
              <p>배터리: 75%</p>
              <p>위치: 구역 A3</p>
              <p>마지막 활동: 10분 전</p>
            </div>
            <div className="mt-4 flex justify-end">
              <button
                onClick={() => handleDetailClick(1)}
                className="px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600 transition-colors"
              >
                상세 정보
              </button>
            </div>
          </div>

          {/* 로봇 #2 카드 */}
          <div className="bg-white p-4 rounded-lg shadow">
            <h2 className="text-lg font-semibold">로봇 #2</h2>
            <div className="flex items-center mt-2">
              <div className="w-3 h-3 rounded-full bg-yellow-500 mr-2"></div>
              <span>대기 중</span>
            </div>
            <div className="mt-3">
              <p>배터리: 45%</p>
              <p>위치: 구역 B1</p>
              <p>마지막 활동: 30분 전</p>
            </div>
            <div className="mt-4 flex justify-end">
              <button
                onClick={() => handleDetailClick(2)}
                className="px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600 transition-colors"
              >
                상세 정보
              </button>
            </div>
          </div>
        </div>

        {/* 모달 영역 */}
        {selectedRobot && (
          <RobotDetailsModal
            isOpen={selectedRobot !== null}
            onClose={handleCloseModal}
            robotId={selectedRobot}
          />
        )}
      </div>
    </Layout>
  );
}

export default RobotPage;
