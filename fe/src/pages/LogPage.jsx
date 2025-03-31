import React, { useState } from 'react';
import Layout from '../components/Layout/Layout';

function LogPage() {
  // 로그 데이터 예시 (실제로는 API에서 가져올 것입니다)
  const [logs, setLogs] = useState([
    { id: 1, timestamp: '2023-06-15 14:23:45', level: 'INFO', message: '로봇 #1 작업 시작', robot: '로봇 #1' },
    { id: 2, timestamp: '2023-06-15 14:25:12', level: 'WARNING', message: '로봇 #2 배터리 부족 (30%)', robot: '로봇 #2' },
    { id: 3, timestamp: '2023-06-15 14:30:45', level: 'ERROR', message: '로봇 #3 연결 끊김', robot: '로봇 #3' },
    { id: 4, timestamp: '2023-06-15 14:35:22', level: 'INFO', message: '로봇 #1 작업 완료', robot: '로봇 #1' },
    { id: 5, timestamp: '2023-06-15 14:40:18', level: 'INFO', message: '로봇 #2 충전 시작', robot: '로봇 #2' },
  ]);
  
  // 로그 레벨에 따른 스타일 클래스
  const getLevelClass = (level) => {
    switch(level) {
      case 'INFO':
        return 'bg-blue-100 text-blue-800';
      case 'WARNING':
        return 'bg-yellow-100 text-yellow-800';
      case 'ERROR':
        return 'bg-red-100 text-red-800';
      default:
        return 'bg-gray-100 text-gray-800';
    }
  };

  return (
    <Layout>
      <div className="p-6">
        <h1 className="text-2xl font-bold mb-4">시스템 로그</h1>
        <p className="mb-6">로봇 및 시스템 활동에 대한 로그를 확인할 수 있습니다.</p>
        
        {/* 필터링 옵션 */}
        <div className="mb-4 flex flex-wrap gap-2">
          <select className="border rounded px-3 py-1 bg-white">
            <option value="">모든 로봇</option>
            <option value="로봇 #1">로봇 #1</option>
            <option value="로봇 #2">로봇 #2</option>
            <option value="로봇 #3">로봇 #3</option>
          </select>
          
          <select className="border rounded px-3 py-1 bg-white">
            <option value="">모든 레벨</option>
            <option value="INFO">INFO</option>
            <option value="WARNING">WARNING</option>
            <option value="ERROR">ERROR</option>
          </select>
          
          <button className="bg-blue-500 text-white px-3 py-1 rounded hover:bg-blue-600">
            필터 적용
          </button>
          
          <button className="bg-gray-300 text-gray-800 px-3 py-1 rounded hover:bg-gray-400">
            초기화
          </button>
        </div>
        
        {/* 로그 테이블 */}
        <div className="overflow-x-auto">
          <table className="min-w-full bg-white border">
            <thead>
              <tr className="bg-gray-100">
                <th className="py-2 px-4 border text-left">시간</th>
                <th className="py-2 px-4 border text-left">로봇</th>
                <th className="py-2 px-4 border text-left">레벨</th>
                <th className="py-2 px-4 border text-left">메시지</th>
              </tr>
            </thead>
            <tbody>
              {logs.map(log => (
                <tr key={log.id} className="hover:bg-gray-50">
                  <td className="py-2 px-4 border">{log.timestamp}</td>
                  <td className="py-2 px-4 border">{log.robot}</td>
                  <td className="py-2 px-4 border">
                    <span className={`px-2 py-1 rounded text-xs font-semibold ${getLevelClass(log.level)}`}>
                      {log.level}
                    </span>
                  </td>
                  <td className="py-2 px-4 border">{log.message}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
        
        {/* 페이지네이션 */}
        <div className="mt-4 flex justify-center">
          <nav className="flex items-center">
            <button className="px-3 py-1 border rounded-l hover:bg-gray-100">&lt;</button>
            <button className="px-3 py-1 border-t border-b bg-blue-500 text-white">1</button>
            <button className="px-3 py-1 border-t border-b hover:bg-gray-100">2</button>
            <button className="px-3 py-1 border-t border-b hover:bg-gray-100">3</button>
            <button className="px-3 py-1 border rounded-r hover:bg-gray-100">&gt;</button>
          </nav>
        </div>
      </div>
    </Layout>
  );
}

export default LogPage;
