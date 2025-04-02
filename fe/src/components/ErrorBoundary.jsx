import React, { Component } from 'react';
import { errorService } from '../services/errorService';

class ErrorBoundary extends Component {
  constructor(props) {
    super(props);
    this.state = { 
      hasError: false,
      error: null,
      errorInfo: null
    };
  }

  static getDerivedStateFromError(error) {
    // 다음 렌더링에서 폴백 UI가 보이도록 상태를 업데이트
    return { hasError: true, error };
  }

  componentDidCatch(error, errorInfo) {
    // 에러 로깅 서비스에 에러 정보 전송
    errorService.logError(error, errorInfo);
    this.setState({ errorInfo });
    
    console.error('ErrorBoundary 캐치:', error, errorInfo);
  }

  handleReload = () => {
    // 페이지 새로고침
    window.location.reload();
  };

  handleGoHome = () => {
    // 홈페이지로 이동
    window.location.href = '/';
  };

  render() {
    if (this.state.hasError) {
      // 사용자 친화적인 폴백 UI
      return (
        <div className="min-h-screen flex items-center justify-center bg-gray-100 px-4">
          <div className="max-w-md w-full bg-white rounded-lg shadow-md p-8">
            <div className="text-center">
              <h2 className="text-2xl font-bold text-red-600 mb-4">
                문제가 발생했습니다
              </h2>
              <div className="mb-4 p-4 bg-red-50 rounded-md">
                <p className="text-gray-700">
                  애플리케이션에서 예상치 못한 오류가 발생했습니다.
                </p>
                {process.env.NODE_ENV === 'development' && this.state.error && (
                  <div className="mt-4 text-left">
                    <p className="font-semibold text-red-700">에러 메시지:</p>
                    <pre className="mt-2 p-2 bg-gray-100 rounded overflow-x-auto text-sm">
                      {this.state.error.toString()}
                    </pre>
                    {this.state.errorInfo && (
                      <>
                        <p className="font-semibold text-red-700 mt-4">컴포넌트 스택:</p>
                        <pre className="mt-2 p-2 bg-gray-100 rounded overflow-x-auto text-sm">
                          {this.state.errorInfo.componentStack}
                        </pre>
                      </>
                    )}
                  </div>
                )}
              </div>
              <div className="flex justify-center space-x-4">
                <button
                  onClick={this.handleReload}
                  className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 focus:outline-none focus:ring-2 focus:ring-blue-300"
                >
                  페이지 새로고침
                </button>
                <button
                  onClick={this.handleGoHome}
                  className="px-4 py-2 bg-gray-500 text-white rounded hover:bg-gray-600 focus:outline-none focus:ring-2 focus:ring-gray-300"
                >
                  홈으로 이동
                </button>
              </div>
            </div>
          </div>
        </div>
      );
    }

    // 에러가 없으면 자식 컴포넌트를 정상적으로 렌더링
    return this.props.children;
  }
}

export default ErrorBoundary;
