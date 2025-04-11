// 에러 로깅 및 처리를 위한 서비스

// 개발 환경에서만 상세 로그 출력 여부
const isVerboseLogging = process.env.NODE_ENV === 'development';

// 에러 심각도 레벨
const ErrorLevel = {
  INFO: 'info',
  WARNING: 'warning',
  ERROR: 'error',
  CRITICAL: 'critical'
};

// 에러 카테고리
const ErrorCategory = {
  NETWORK: 'network',
  AUTH: 'authentication',
  API: 'api',
  UI: 'ui',
  UNKNOWN: 'unknown'
};

/**
 * 에러 서비스 객체
 */
export const errorService = {
  /**
   * 에러 로깅 함수
   * @param {Error} error - 발생한 에러 객체
   * @param {Object} errorInfo - 추가 에러 정보 (React ErrorBoundary에서 제공하는 정보 등)
   * @param {string} level - 에러 심각도 (기본값: 'error')
   * @param {string} category - 에러 카테고리 (기본값: 'unknown')
   */
  logError: (error, errorInfo = {}, level = ErrorLevel.ERROR, category = ErrorCategory.UNKNOWN) => {
    // 콘솔에 에러 로깅
    if (isVerboseLogging) {
      console.group(`[${level.toUpperCase()}] ${category} 에러`);
      console.error('에러:', error);
      if (errorInfo.componentStack) {
        console.error('컴포넌트 스택:', errorInfo.componentStack);
      }
      console.groupEnd();
    } else {
      console.error(`[${level}] ${category} 에러:`, error.message);
    }

    // 프로덕션 환경에서는 서버에 에러 로깅을 전송할 수 있음
    if (process.env.NODE_ENV === 'production') {
      try {
        // 여기에 외부 에러 추적 서비스(Sentry 등)로 에러 전송 로직 구현
        // 예: sendErrorToServer(error, errorInfo, level, category);
      } catch (loggingError) {
        // 에러 로깅 자체에서 에러가 발생한 경우 조용히 처리
        console.error('에러 로깅 중 추가 에러 발생:', loggingError);
      }
    }

    // 나중에 분석을 위해 에러 정보 반환
    return {
      timestamp: new Date().toISOString(),
      error: {
        name: error.name,
        message: error.message,
        stack: error.stack
      },
      info: errorInfo,
      level,
      category
    };
  },

  /**
   * 네트워크 에러 처리 함수
   * @param {Error} error - 발생한 에러 객체
   * @param {Object} requestInfo - 요청 정보
   */
  handleNetworkError: (error, requestInfo = {}) => {
    const category = ErrorCategory.NETWORK;
    let level = ErrorLevel.ERROR;
    let userMessage = '네트워크 연결 중 문제가 발생했습니다.';

    // 에러 상태 코드에 따른 처리
    if (error.response) {
      // 서버 응답이 있는 경우
      const status = error.response.status;

      if (status === 401 || status === 403) {
        level = ErrorLevel.WARNING;
        userMessage = '인증에 실패했습니다. 다시 로그인해주세요.';
      } else if (status === 404) {
        level = ErrorLevel.WARNING;
        userMessage = '요청한 리소스를 찾을 수 없습니다.';
      } else if (status >= 500) {
        level = ErrorLevel.CRITICAL;
        userMessage = '서버에 문제가 발생했습니다. 잠시 후 다시 시도해주세요.';
      }
    } else if (error.request) {
      // 요청은 보냈으나 응답이 없는 경우
      level = ErrorLevel.ERROR;
      userMessage = '서버에 연결할 수 없습니다. 네트워크 연결을 확인해주세요.';
    }

    // 에러 로깅
    errorService.logError(error, requestInfo, level, category);

    return { level, message: userMessage, category };
  },

  /**
   * 인증 에러 처리 함수
   * @param {Error} error - 발생한 에러 객체
   */
  handleAuthError: (error) => {
    const category = ErrorCategory.AUTH;
    let level = ErrorLevel.WARNING;
    let userMessage = '인증에 실패했습니다. 다시 로그인해주세요.';

    // 에러 로깅
    errorService.logError(error, {}, level, category);

    return { level, message: userMessage, category };
  },

  /**
   * API 에러 처리 함수
   * @param {Error} error - 발생한 에러 객체
   * @param {Object} apiInfo - API 요청 정보
   */
  handleApiError: (error, apiInfo = {}) => {
    const category = ErrorCategory.API;
    let level = ErrorLevel.ERROR;
    let userMessage = '데이터를 처리하는 중 오류가 발생했습니다.';

    // 에러 로깅
    errorService.logError(error, apiInfo, level, category);

    return { level, message: userMessage, category };
  },

  /**
   * UI 에러 처리 함수
   * @param {Error} error - 발생한 에러 객체
   * @param {Object} componentInfo - 컴포넌트 정보
   */
  handleUiError: (error, componentInfo = {}) => {
    const category = ErrorCategory.UI;
    let level = ErrorLevel.ERROR;
    let userMessage = '화면을 표시하는 중 오류가 발생했습니다.';

    // 에러 로깅
    errorService.logError(error, componentInfo, level, category);

    return { level, message: userMessage, category };
  }
};

// 에러 레벨과 카테고리 내보내기
export { ErrorLevel, ErrorCategory };
