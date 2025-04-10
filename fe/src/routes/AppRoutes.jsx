import React, { Suspense, lazy, useEffect, useState } from 'react';
import { Routes, Route, Navigate, useNavigate } from 'react-router-dom';
import { useAuthStore } from '../stores/authStore';
import ErrorBoundary from '../components/ErrorBoundary';
import AnnouncementForm from '../components/board/AnnouncementForm';

// 지연 로딩을 사용한 페이지 컴포넌트 임포트
const LoginPageComponent = lazy(() => import('../pages/LoginPage'));
const RegisterPageComponent = lazy(() => import('../pages/RegisterPage'));
const MapPageComponent = lazy(() => import('../pages/MapPage'));
const RobotPageComponent = lazy(() => import('../pages/RobotPage'));
const InventoryPageComponent = lazy(() => import('../pages/InventoryPage'));
const InventoryDetailPageComponent = lazy(() => import('../pages/InventoryDetailPage'));
const SettingsPageComponent = lazy(() => import('../pages/SettingsPage'));
const WorkPageComponent = lazy(() => import('../pages/WorkPage'));
const AnnouncementDetailComponent = lazy(() =>
  import('../components/board/AnnouncementDetail')
);
const AnnouncementListComponent = lazy(() =>
  import('../components/board/AnnouncementList')
);

// 로딩 컴포넌트
const LoadingFallback = () => (
  <div className="flex items-center justify-center h-screen">
    <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
  </div>
);

// 보호된 라우트 컴포넌트
const ProtectedRoute = ({ children }) => {
  const { isAuthenticated } = useAuthStore();
  const navigate = useNavigate();
  const [isRedirecting, setIsRedirecting] = useState(false);

  useEffect(() => {
    if (!isAuthenticated() && !isRedirecting) {
      setIsRedirecting(true);
      // 약간의 지연을 주어 네비게이션 제한 문제를 방지
      setTimeout(() => {
        navigate('/login', { replace: true });
      }, 100);
    }
  }, [isAuthenticated, navigate, isRedirecting]);

  if (!isAuthenticated()) {
    return <LoadingFallback />;
  }

  return children;
};

// 공개 라우트 컴포넌트 - 이미 인증된 사용자는 메인 페이지로 리다이렉트 (선택적)
const PublicRoute = ({ children }) => {
  const { isAuthenticated } = useAuthStore();
  const navigate = useNavigate();
  const [isRedirecting, setIsRedirecting] = useState(false);

  useEffect(() => {
    if (isAuthenticated() && !isRedirecting) {
      setIsRedirecting(true);
      // 약간의 지연을 주어 네비게이션 제한 문제를 방지
      setTimeout(() => {
        navigate('/board', { replace: true });
      }, 100);
    }
  }, [isAuthenticated, navigate, isRedirecting]);

  if (isAuthenticated()) {
    return <LoadingFallback />;
  }

  return children;
};

function AppRoutes() {
  return (
    <ErrorBoundary>
      <Suspense fallback={<LoadingFallback />}>
        <Routes>
          {/* 공개 라우트 - 인증이 필요하지 않음 */}
          <Route
            path="/login"
            element={
              <PublicRoute>
                <ErrorBoundary>
                  <LoginPageComponent />
                </ErrorBoundary>
              </PublicRoute>
            }
          />
          <Route
            path="/register"
            element={
              <PublicRoute>
                <ErrorBoundary>
                  <RegisterPageComponent />
                </ErrorBoundary>
              </PublicRoute>
            }
          />

          {/* 보호된 라우트 - 인증이 필요함 */}
          <Route
            path="/board"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <AnnouncementListComponent />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          />
          <Route
            path="/map"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <MapPageComponent />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          />
          <Route
            path="/robot"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <RobotPageComponent />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          />
          <Route
            path="/move"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <WorkPageComponent />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          />
          <Route
            path="/inventory"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <InventoryPageComponent />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          />
          <Route
            path="/inventory/:itemId/history"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <InventoryDetailPageComponent />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          />
          <Route
            path="/settings"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <SettingsPageComponent />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          />

          <Route
            path="/board/write"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <AnnouncementForm />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          />

          <Route
            path="/board/edit/:id"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <AnnouncementForm />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          />

          <Route
            path="/board/:id"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <AnnouncementDetailComponent />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          />

          {/* 기본 및 404 리다이렉션 */}
          <Route path="/" element={<Navigate to="/board" replace />} />
          <Route
            path="*"
            element={
              <ErrorBoundary>
                <div className="flex flex-col items-center justify-center h-screen">
                  <h1 className="text-3xl font-bold text-gray-700 mb-4">
                    404 - 페이지를 찾을 수 없습니다
                  </h1>
                  <p className="text-gray-600 mb-6">
                    요청하신 페이지가 존재하지 않습니다.
                  </p>
                  <button
                    onClick={() => (window.location.href = '/board')}
                    className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
                  >
                    공지 페이지로 이동
                  </button>
                </div>
              </ErrorBoundary>
            }
          />
        </Routes>
      </Suspense>
    </ErrorBoundary>
  );
}

export default AppRoutes;
