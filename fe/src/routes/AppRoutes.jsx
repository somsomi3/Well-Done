import React, { Suspense, lazy } from 'react';
import { Routes, Route, Navigate } from 'react-router-dom';
import { useAuthStore } from '../stores/authStore';
import ErrorBoundary from '../components/ErrorBoundary';

// 지연 로딩을 사용한 페이지 컴포넌트 임포트
const LoginPage = lazy(() => import('../pages/LoginPage'));
const RegisterPage = lazy(() => import('../pages/RegisterPage'));
const MainPage = lazy(() => import('../pages/MainPage'));
const MapPage = lazy(() => import('../pages/MapPage'));
const RobotPage = lazy(() => import('../pages/RobotPage'));
const LogPage = lazy(() => import('../pages/LogPage'));
const SettingsPage = lazy(() => import('../pages/SettingsPage'));

// 로딩 컴포넌트
const LoadingFallback = () => (
  <div className="flex items-center justify-center h-screen">
    <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
  </div>
);

// 보호된 라우트 컴포넌트
const ProtectedRoute = ({ children }) => {
  const { isAuthenticated } = useAuthStore();
  
  if (!isAuthenticated()) {
    return <Navigate to="/login" replace />;
  }
  
  return children;
};

function AppRoutes() {
  return (
    <ErrorBoundary>
      <Suspense fallback={<LoadingFallback />}>
        <Routes>
          {/* 공개 라우트 */}
          <Route path="/login" element={<LoginPage />} />
          <Route path="/register" element={<RegisterPage />} />
          
          {/* 보호된 라우트 */}
          <Route 
            path="/main" 
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <MainPage />
                </ErrorBoundary>
              </ProtectedRoute>
            } 
          />
          <Route 
            path="/map" 
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <MapPage />
                </ErrorBoundary>
              </ProtectedRoute>
            } 
          />
          <Route 
            path="/robot" 
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <RobotPage />
                </ErrorBoundary>
              </ProtectedRoute>
            } 
          />
          <Route 
            path="/log" 
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <LogPage />
                </ErrorBoundary>
              </ProtectedRoute>
            } 
          />
          <Route 
            path="/settings" 
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <SettingsPage />
                </ErrorBoundary>
              </ProtectedRoute>
            } 
          />
          
          {/* 기본 및 404 리다이렉션 */}
          <Route path="/" element={<Navigate to="/main" replace />} />
          <Route path="*" element={
            <ErrorBoundary>
              <div className="flex flex-col items-center justify-center h-screen">
                <h1 className="text-3xl font-bold text-gray-700 mb-4">404 - 페이지를 찾을 수 없습니다</h1>
                <p className="text-gray-600 mb-6">요청하신 페이지가 존재하지 않습니다.</p>
                <button 
                  onClick={() => window.location.href = '/main'}
                  className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
                >
                  메인 페이지로 이동
                </button>
              </div>
            </ErrorBoundary>
          } />
        </Routes>
      </Suspense>
    </ErrorBoundary>
  );
}

export default AppRoutes;
