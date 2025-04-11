import React from 'react';
import Layout from '../components/Layout/Layout';

function MainPage() {
  return (
    <Layout>
      <div className="p-6">
        <h1 className="text-2xl font-bold mb-4">메인 페이지</h1>
        <p>로그인 성공! 메인 페이지에 오신 것을 환영합니다.</p>
      </div>
    </Layout>
  );
}

export default MainPage;
