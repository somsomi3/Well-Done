import React, { useState, useEffect } from 'react';
import { useNavigate, useParams } from 'react-router-dom';
import Layout from '../Layout/Layout';
import { api } from '../../utils/api';
import './AnnouncementForm.css';

function AnnouncementForm() {
  const navigate = useNavigate();
  const { id } = useParams();
  const isEditMode = !!id;

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
    const fetchAnnouncementData = async () => {
      if (isEditMode) {
        try {
          const response = await api.get(`/boards/announcements/${id}`);
          const announcement = response.data;
          setFormData({
            title: announcement.title,
            content: announcement.content,
            writer: announcement.writer,
            expirationDate: announcement.expirationDate
              ? new Date(announcement.expirationDate).toISOString().slice(0, 16)
              : new Date(Date.now() + 7 * 24 * 60 * 60 * 1000)
                  .toISOString()
                  .slice(0, 16),
          });
        } catch (error) {
          console.error('공지사항 조회 실패:', error);
          navigate('/board');
        }
      }
    };

    // 토큰에서 사용자 정보 추출
    const token = localStorage.getItem('accessToken');
    if (token) {
      try {
        const decodedToken = JSON.parse(atob(token.split('.')[1]));
        if (!isEditMode) {
          setFormData((prev) => ({
            ...prev,
            writer: decodedToken.username,
          }));
        }

        // 사용자 역할 확인
        const isUserAdmin = decodedToken.userId === 1;
        setIsAdmin(isUserAdmin);

        // 관리자가 아닌 경우 공지사항 목록 페이지로 리다이렉트
        if (!isUserAdmin) {
          console.error('관리자 권한이 필요합니다.');
          navigate('/board');
        } else {
          fetchAnnouncementData();
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
  }, [navigate, id, isEditMode]);

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

      let response;
      if (isEditMode) {
        response = await api.patch(`/boards/announcements/${id}`, formData);
        console.log('공지사항 수정 성공:', response.data);
      } else {
        response = await api.post('/boards/announcements', formData);
        console.log('공지사항 생성 성공:', response.data);
      }

      // 공지사항 목록을 다시 불러오기
      try {
        const listResponse = await api.get('/boards/announcements');
        console.log('공지사항 목록:', listResponse.data);
      } catch (listError) {
        console.error('공지사항 목록 조회 실패:', listError);
      }

      // 공지사항 목록 페이지로 리다이렉트
      navigate('/board', { replace: true });
    } catch (error) {
      console.error(
        isEditMode ? '공지사항 수정 실패:' : '공지사항 생성 실패:',
        error
      );
      if (error.response?.status === 403) {
        console.error('인증이 필요합니다. 로그인 페이지로 이동합니다.');
        navigate('/');
      }
    }
  };

  // 로딩 중이거나 관리자가 아닌 경우 로딩 화면 표시
  if (isLoading || !isAdmin) {
    return (
      <Layout>
        <div className="loading-container">
          <div className="loading-spinner"></div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout>
      <div className="write-page">
        <div className="write-container">
          <div className="write-header">
            <h1 className="write-title">
              <span className="write-title-gradient">
                {isEditMode ? '공지사항 수정' : '공지사항 작성'}
              </span>
            </h1>
          </div>

          <div className="form-container">
            <form onSubmit={handleSubmit} className="announcement-form">
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
                  rows="10"
                />
              </div>

              <div className="form-row">
                <div className="form-group half-width">
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

                <div className="form-group half-width">
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
              </div>

              <div className="button-group">
                <button
                  type="button"
                  onClick={() => navigate('/board')}
                  className="cancel-button"
                >
                  취소
                </button>
                <button type="submit" className="submit-button">
                  {isEditMode ? '수정' : '작성'}
                </button>
              </div>
            </form>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default AnnouncementForm;
