import React, { useState, useEffect } from 'react';
import Layout from '../Layout/Layout';
import { useNavigate, useParams } from 'react-router-dom';
import { api } from '../../utils/api';
import './AnnouncementDetail.css';

function AnnouncementDetail() {
  const navigate = useNavigate();
  const { id } = useParams();
  const [announcement, setAnnouncement] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [isAdmin, setIsAdmin] = useState(false);

  useEffect(() => {
    const fetchAnnouncement = async () => {
      try {
        const token = localStorage.getItem('accessToken');
        if (!token) {
          navigate('/');
          return;
        }

        // 게시글 정보 가져오기
        const response = await api.get(`/boards/announcements/${id}`);
        setAnnouncement(response.data);
      } catch (error) {
        console.error('공지사항 조회 실패:', error);
        if (error.response?.status === 403) {
          navigate('/');
        }
      } finally {
        setIsLoading(false);
      }
    };

    const checkUserRole = () => {
      const token = localStorage.getItem('accessToken');
      if (token) {
        try {
          const decodedToken = JSON.parse(atob(token.split('.')[1]));
          setIsAdmin(decodedToken.userId === 1);
        } catch (error) {
          console.error('토큰 디코딩 오류:', error);
        }
      }
    };

    fetchAnnouncement();
    checkUserRole();
  }, [id, navigate]);

  const handleDelete = async () => {
    if (!window.confirm('정말로 이 공지사항을 삭제하시겠습니까?')) {
      return;
    }

    try {
      const token = localStorage.getItem('accessToken');
      if (!token) {
        navigate('/');
        return;
      }

      await api.delete(`/boards/announcements/${id}`);
      navigate('/board');
    } catch (error) {
      console.error('공지사항 삭제 실패:', error);
      if (error.response?.status === 403) {
        alert('삭제 권한이 없습니다.');
      }
    }
  };

  if (isLoading) {
    return (
      <Layout>
        <div className="loading-container">
          <div className="loading-spinner"></div>
        </div>
      </Layout>
    );
  }

  if (!announcement) {
    return (
      <Layout>
        <div className="error-message">공지사항을 찾을 수 없습니다.</div>
      </Layout>
    );
  }

  return (
    <Layout>
      <div className="detail-page">
        <div className="detail-container">
          <div className="detail-header">
            <h1 className="detail-title">
              <span className="detail-title-gradient">공지사항 상세</span>
            </h1>
          </div>

          <div className="detail-content">
            <div className="detail-card">
              <div className="detail-info">
                <div className="detail-title-section">
                  <h2 className="announcement-title">{announcement.title}</h2>
                  <div className="detail-meta">
                    <span className="writer">
                      작성자: {announcement.writer}
                    </span>
                    <span className="date">
                      작성일:{' '}
                      {announcement.createdAt
                        ? new Date(announcement.createdAt).toLocaleDateString()
                        : '날짜 정보 없음'}
                    </span>
                    <span className="date">
                      수정일:{' '}
                      {announcement.updatedAt &&
                      announcement.updatedAt !== announcement.createdAt &&
                      new Date(announcement.updatedAt).getTime() >
                        new Date(announcement.createdAt).getTime()
                        ? new Date(announcement.updatedAt).toLocaleString(
                            'ko-KR',
                            {
                              year: 'numeric',
                              month: '2-digit',
                              day: '2-digit',
                              hour: '2-digit',
                              minute: '2-digit',
                              hour12: false,
                            }
                          )
                        : '수정 없음'}
                    </span>
                  </div>
                </div>

                <div className="detail-body">
                  <p className="content">{announcement.content}</p>
                </div>
              </div>

              <div className="detail-actions">
                <button
                  onClick={() => navigate('/board')}
                  className="back-button"
                >
                  목록으로
                </button>
                {isAdmin && (
                  <>
                    <button
                      onClick={() => navigate(`/board/edit/${id}`)}
                      className="edit-button"
                    >
                      수정
                    </button>
                    <button onClick={handleDelete} className="delete-button">
                      삭제
                    </button>
                  </>
                )}
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default AnnouncementDetail;
