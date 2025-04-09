import React, { Suspense, lazy, useEffect, useState } from 'react';
import { Routes, Route, Navigate, useNavigate } from 'react-router-dom';
import { useAuthStore } from '../stores/authStore';
import ErrorBoundary from '../components/ErrorBoundary';
import MainPage from '../pages/MainPage';
import LoginPage from '../pages/LoginPage';
import RegisterPage from '../pages/RegisterPage';
import MapPage from '../pages/MapPage';
import RobotPage from '../pages/RobotPage';
import LogPage from '../pages/LogPage';
import SettingsPage from '../pages/SettingsPage';
import AnnouncementDetail from '../components/board/AnnouncementDetail';
import AnnouncementList from '../components/board/AnnouncementList';
import AnnouncementForm from '../components/board/AnnouncementForm';

// 지연 로딩을 사용한 페이지 컴포넌트 임포트
const LoginPageComponent = lazy(() => import('../pages/LoginPage'));
const RegisterPageComponent = lazy(() => import('../pages/RegisterPage'));
const MainPageComponent = lazy(() => import('../pages/MainPage'));
const MapPageComponent = lazy(() => import('../pages/MapPage'));
const RobotPageComponent = lazy(() => import('../pages/RobotPage'));
const LogPageComponent = lazy(() => import('../pages/LogPage'));
const SettingsPageComponent = lazy(() => import('../pages/SettingsPage'));
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
        navigate('/main', { replace: true });
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
          {/* <Route
            path="/main"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <MainPageComponent />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          /> */}
          <Route
            path="/main"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <Navigate to="/board" replace />
                </ErrorBoundary>
              </ProtectedRoute>
            }
          />
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
            path="/log"
            element={
              <ProtectedRoute>
                <ErrorBoundary>
                  <LogPageComponent />
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
          <Route path="/" element={<Navigate to="/main" replace />} />
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
                    onClick={() => (window.location.href = '/main')}
                    className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
                  >
                    메인 페이지로 이동
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
