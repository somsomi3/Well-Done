import React from 'react';
import { Routes, Route, Navigate } from 'react-router-dom';
import LoginPage from '../pages/LoginPage';
import RegisterPage from '../pages/RegisterPage';
import MainPage from '../pages/MainPage';
import MapPage from '../pages/MapPage';
import RobotPage from '../pages/RobotPage';
import LogPage from '../pages/LogPage';
import SettingsPage from '../pages/SettingsPage';
import { useAuthStore } from '../stores/authStore';

// 보호된 라우트 컴포넌트
const ProtectedRoute = ({ children }) => {
  const { token } = useAuthStore();
  
  if (!token) {
    return <Navigate to="/login" replace />;
  }
  
  return children;
};

function AppRoutes() {
  return (
    <Routes>
      {/* 공개 라우트 */}
      <Route path="/login" element={<LoginPage />} />
      <Route path="/register" element={<RegisterPage />} />
      
      {/* 보호된 라우트 */}
      <Route 
        path="/main" 
        element={
          <ProtectedRoute>
            <MainPage />
          </ProtectedRoute>
        } 
      />
      <Route 
        path="/map" 
        element={
          <ProtectedRoute>
            <MapPage />
          </ProtectedRoute>
        } 
      />
      <Route 
        path="/robot" 
        element={
          <ProtectedRoute>
            <RobotPage />
          </ProtectedRoute>
        } 
      />
      <Route 
        path="/log" 
        element={
          <ProtectedRoute>
            <LogPage />
          </ProtectedRoute>
        } 
      />
      <Route 
        path="/settings" 
        element={
          <ProtectedRoute>
            <SettingsPage />
          </ProtectedRoute>
        } 
      />
      
      {/* 기본 및 404 리다이렉션 */}
      <Route path="/" element={<Navigate to="/main" replace />} />
      <Route path="*" element={<Navigate to="/main" replace />} />
    </Routes>
  );
}

export default AppRoutes;
