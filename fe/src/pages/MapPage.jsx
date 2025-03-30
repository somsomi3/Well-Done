import React from 'react';
import Layout from '../components/Layout/Layout';

function MapPage() {
  return (
    <Layout>
      <div className="p-6">
        <h1 className="text-2xl font-bold mb-4">맵 페이지</h1>
        <p>로봇 위치 및 경로를 확인할 수 있는 맵 페이지입니다.</p>
        
        {/* 맵 컴포넌트가 여기에 추가될 수 있습니다 */}
        <div className="mt-4 p-4 border-2 border-dashed border-gray-300 rounded-lg h-96 flex items-center justify-center bg-gray-50">
          <p className="text-gray-500">맵 영역 - 추후 실제 맵 컴포넌트로 대체될 예정입니다.</p>
        </div>
      </div>
    </Layout>
  );
}

export default MapPage;
