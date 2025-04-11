import React, { useState } from 'react';
import Layout from '../components/Layout/Layout';
import RobotCard from '../components/Robot/RobotCard';
import RobotDetailsModal from '../components/Robot/RobotDetailsModal';
import styles from '../styles/RobotPage.module.css'; // CSS 모듈 import

function RobotPage() {
  const [selectedRobot, setSelectedRobot] = useState(null);

  // 로봇 데이터 (실제로는 API에서 가져올 수 있음)
  const robots = [
    {
      id: 1,
      status: 'active',
      battery: 75,
      location: '구역 A-3',
      lastActivity: '10분 전'
    },
    {
      id: 2,
      status: 'standby',
      battery: 45,
      location: '구역 B-1',
      lastActivity: '30분 전'
    }
  ];

  // 상세 정보 버튼 클릭 핸들러
  const handleDetailClick = (robotId) => {
    setSelectedRobot(robotId);
  };

  // 모달 닫기 핸들러
  const handleCloseModal = () => {
    setSelectedRobot(null);
  };

  return (
    <Layout>
      {/* 중앙 정렬을 위한 컨테이너 - CSS 모듈 적용 */}
      <div className="flex justify-center">
        <div className={styles.container}>
          {/* 작업 현황 헤더 */}
          <h2 className={styles.sectionHeader}>작업 현황</h2>
          
          {/* 상태 요약 - 중앙 정렬 유지 */}
          <div className={styles.statusGrid}>
            <div className={styles.statusCard}>
              <div className={styles.iconContainer}>
                <svg xmlns="http://www.w3.org/2000/svg" className={styles.icon} fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
                </svg>
              </div>
              <p className={styles.statusLabel}>활성 로봇</p>
              <p className={styles.statusValue}>1</p>
            </div>

            <div className={styles.statusCard}>
              <div className={styles.iconContainer}>
                <svg xmlns="http://www.w3.org/2000/svg" className={styles.icon} fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
                </svg>
              </div>
              <p className={styles.statusLabel}>대기 중</p>
              <p className={styles.statusValue}>1</p>
            </div>

            <div className={styles.statusCard}>
              <div className={styles.iconContainer}>
                <svg xmlns="http://www.w3.org/2000/svg" className={styles.icon} fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
              </div>
              <p className={styles.statusLabel}>총 작업 수</p>
              <p className={styles.statusValue}>24</p>
            </div>
          </div>

          {/* 로봇 목록 - 중앙 정렬 제목 */}
          <h2 className={styles.sectionHeader}>로봇 목록</h2>
          
          {/* 로봇 카드 - 3열 그리드 기준 정렬 */}
          <div className={styles.robotGrid}>
            {/* 로봇 카드 2개만 표시 (첫 2칸) */}
            <div className={styles.gridItem}>
              <div className={styles.cardContainer}>
                <RobotCard
                  id={robots[0].id}
                  status={robots[0].status}
                  battery={robots[0].battery}
                  location={robots[0].location}
                  lastActivity={robots[0].lastActivity}
                  onDetailClick={handleDetailClick}
                />
              </div>
            </div>
            <div className={styles.gridItem}>
              <div className={styles.cardContainer}>
                <RobotCard
                  id={robots[1].id}
                  status={robots[1].status}
                  battery={robots[1].battery}
                  location={robots[1].location}
                  lastActivity={robots[1].lastActivity}
                  onDetailClick={handleDetailClick}
                />
              </div>
            </div>
            
            {/* 로봇 추가 버튼 - 점선 테두리 박스 */}
            <div className={styles.gridItem}>
              <div className={styles.addRobotCard}>
                <div className={styles.addIconContainer}>
                  <svg xmlns="http://www.w3.org/2000/svg" className={styles.addIcon} fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 4v16m8-8H4" />
                  </svg>
                </div>
                <p className={styles.addRobotText}>로봇 추가</p>
              </div>
            </div>
          </div>

          {/* 모달 */}
          {selectedRobot && (
            <RobotDetailsModal
              isOpen={selectedRobot !== null}
              onClose={handleCloseModal}
              robotId={selectedRobot}
            />
          )}
        </div>
      </div>
    </Layout>
  );
}

export default RobotPage;
