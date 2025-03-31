import axios from 'axios';

// API 기본 URL 설정
const baseURL = import.meta.env.VITE_BASE_URL || 'https://j12e102.p.ssafy.io/api';

const api = axios.create({
  baseURL,
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  }
});

const publicApi = axios.create({
  baseURL,
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  }
});

// 인증이 필요한 요청에만 토큰 추가
api.interceptors.request.use(
  (config) => {
    console.log('API 요청 설정:', {
      url: `${config.baseURL}${config.url}`,
      method: config.method,
      headers: config.headers
    });
    
    const token = localStorage.getItem('accessToken');
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  (error) => {
    console.error('API 요청 오류:', error);
    return Promise.reject(error);
  }
);

// 인증이 필요없는 요청은 토큰 추가하지 않음
publicApi.interceptors.request.use(
  (config) => {
    console.log('Public API 요청 설정:', {
      url: `${config.baseURL}${config.url}`,
      method: config.method,
      data: config.data
    });
    return config;
  },
  (error) => {
    console.error('Public API 요청 오류:', error);
    return Promise.reject(error);
  }
);

// 응답 인터셉터 추가
api.interceptors.response.use(
  (response) => {
    console.log('API 응답 성공:', {
      status: response.status,
      url: `${response.config.baseURL}${response.config.url}`,
      data: response.data
    });
    return response;
  },
  (error) => {
    if (error.response) {
      console.error('API 응답 오류:', {
        status: error.response.status,
        statusText: error.response.statusText,
        url: `${error.config.baseURL}${error.config.url}`,
        data: error.response.data
      });
    } else if (error.request) {
      console.error('API 응답을 받지 못했습니다:', {
        request: error.request,
        message: '서버에 연결할 수 없거나 응답이 없습니다.'
      });
    } else {
      console.error('API 요청 설정 중 오류:', error.message);
    }
    
    return Promise.reject(error);
  }
);

// publicApi에도 동일한 응답 인터셉터 추가
publicApi.interceptors.response.use(
  (response) => {
    console.log('Public API 응답 성공:', {
      status: response.status,
      url: `${response.config.baseURL}${response.config.url}`,
      data: response.data
    });
    return response;
  },
  (error) => {
    if (error.response) {
      console.error('Public API 응답 오류:', {
        status: error.response.status,
        statusText: error.response.statusText,
        url: `${error.config.baseURL}${error.config.url}`,
        data: error.response.data
      });
    } else if (error.request) {
      console.error('Public API 응답을 받지 못했습니다:', {
        message: '서버에 연결할 수 없거나 응답이 없습니다.'
      });
    } else {
      console.error('Public API 요청 설정 중 오류:', error.message);
    }
    
    return Promise.reject(error);
  }
);

export { api, publicApi };
