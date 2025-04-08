import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { useNavigate } from 'react-router-dom';
import './AnnouncementForm.css';

function AnnouncementForm() {
  const navigate = useNavigate();
  const [formData, setFormData] = useState({
    title: '',
    content: '',
    writer: '',
    expirationDate: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000)
      .toISOString()
      .slice(0, 16),
  });
  const [isAdmin, setIsAdmin] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // 토큰에서 사용자 정보 추출
    const token = localStorage.getItem('accessToken');
    if (token) {
      try {
        const decodedToken = JSON.parse(atob(token.split('.')[1]));
        setFormData((prev) => ({
          ...prev,
          writer: decodedToken.username,
        }));

        // 사용자 역할 확인
        const isUserAdmin = decodedToken.userId === 1;
        setIsAdmin(isUserAdmin);

        // 관리자가 아닌 경우 공지사항 목록 페이지로 리다이렉트
        if (!isUserAdmin) {
          console.error('관리자 권한이 필요합니다.');
          navigate('/announcements');
        }
      } catch (error) {
        console.error('토큰 디코딩 오류:', error);
        navigate('/');
      }
    } else {
      // 토큰이 없는 경우 로그인 페이지로 리다이렉트
      navigate('/');
    }

    setIsLoading(false);
  }, [navigate]);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      const token = localStorage.getItem('accessToken');
      if (!token) {
        console.error('토큰이 없습니다. 로그인이 필요합니다.');
        navigate('/');
        return;
      }

      const response = await axios.post(
        'http://localhost:8080/api/boards/announcements',
        formData,
        {
          headers: {
            Authorization: `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
        }
      );

      console.log('공지사항 생성 성공:', response.data);

      // 공지사항 목록을 다시 불러오기
      try {
        const listResponse = await axios.get(
          'http://localhost:8080/api/boards/announcements',
          {
            headers: {
              Authorization: `Bearer ${token}`,
            },
          }
        );
        console.log('공지사항 목록:', listResponse.data);
      } catch (listError) {
        console.error('공지사항 목록 조회 실패:', listError);
      }

      // 공지사항 목록 페이지로 리다이렉트
      navigate('/announcements');
    } catch (error) {
      console.error('공지사항 생성 실패:', error);
      if (error.response?.status === 403) {
        console.error('인증이 필요합니다. 로그인 페이지로 이동합니다.');
        navigate('/');
      }
    }
  };

  // 로딩 중이거나 관리자가 아닌 경우 로딩 화면 표시
  if (isLoading || !isAdmin) {
    return (
      <div className="loading-container">
        <div className="loading-spinner"></div>
      </div>
    );
  }

  return (
    <div className="write-page">
      <div className="write-container">
        <div className="write-header">
          <h1 className="write-title">
            <span className="write-title-gradient">공지사항 작성</span>
          </h1>
        </div>

        <div className="form-container">
          <form onSubmit={handleSubmit}>
            <div className="form-group">
              <label htmlFor="title" className="form-label">
                제목
              </label>
              <input
                type="text"
                id="title"
                name="title"
                value={formData.title}
                onChange={handleChange}
                required
                className="form-input"
                placeholder="제목을 입력하세요"
              />
            </div>

            <div className="form-group">
              <label htmlFor="content" className="form-label">
                내용
              </label>
              <textarea
                id="content"
                name="content"
                value={formData.content}
                onChange={handleChange}
                required
                className="form-textarea"
                placeholder="내용을 입력하세요"
              />
            </div>

            <div className="form-group">
              <label htmlFor="writer" className="form-label">
                작성자
              </label>
              <input
                type="text"
                id="writer"
                name="writer"
                value={formData.writer}
                readOnly
                className="form-input readonly"
              />
            </div>

            <div className="form-group">
              <label htmlFor="expirationDate" className="form-label">
                만료일
              </label>
              <input
                type="datetime-local"
                id="expirationDate"
                name="expirationDate"
                value={
                  formData.expirationDate
                    ? formData.expirationDate.slice(0, 16)
                    : ''
                }
                onChange={handleChange}
                className="form-input"
              />
              <small className="form-text text-muted">
                만료일을 설정하지 않으면 공지사항이 계속 표시됩니다.
              </small>
            </div>

            <div className="button-group">
              <button
                type="button"
                onClick={() => navigate('/announcements')}
                className="cancel-button"
              >
                취소
              </button>
              <button type="submit" className="submit-button">
                작성
              </button>
            </div>
          </form>
        </div>
      </div>
    </div>
  );
}

export default AnnouncementForm;
