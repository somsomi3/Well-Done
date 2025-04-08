import React, { useState, useEffect } from 'react';
import Layout from '../Layout/Layout';
import axios from 'axios';
import { useNavigate, useParams } from 'react-router-dom';
import './AnnouncementDetail.css';

function AnnouncementDetail() {
  const navigate = useNavigate();
  const { id } = useParams();
  const [announcement, setAnnouncement] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [isAdmin, setIsAdmin] = useState(false);
  const [viewCountIncreased, setViewCountIncreased] = useState(false);

  useEffect(() => {
    const fetchAnnouncement = async () => {
      try {
        const token = localStorage.getItem('accessToken');
        if (!token) {
          navigate('/');
          return;
        }

        // 게시글 정보 가져오기
        const response = await axios.get(
          `http://localhost:8080/api/boards/announcements/${id}`,
          {
            headers: {
              Authorization: `Bearer ${token}`,
            },
          }
        );

        setAnnouncement(response.data);

        // 조회수 증가 API 별도 호출 (한 번만 호출되도록)
        if (!viewCountIncreased) {
          try {
            // Authorization 헤더 없이 요청하기
            await axios.post(
              `http://localhost:8080/api/boards/announcements/${id}/view`
            );
            setViewCountIncreased(true);
          } catch (error) {
            console.warn('조회수 증가 실패:', error);
            // 오류가 발생해도 계속 진행 (에러 무시)
            setViewCountIncreased(true); // 중복 호출 방지를 위해 true로 설정
          }
        }
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
  }, [id, navigate, viewCountIncreased]);

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

      await axios.delete(
        `http://localhost:8080/api/boards/announcements/${id}`,
        {
          headers: {
            Authorization: `Bearer ${token}`,
          },
        }
      );

      navigate('/announcements');
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
                      {new Date(announcement.createdAt).toLocaleDateString()}
                    </span>
                    <span className="views">
                      조회수: {announcement.viewCount}
                    </span>
                  </div>
                </div>

                <div className="detail-body">
                  <p className="content">{announcement.content}</p>
                </div>
              </div>

              <div className="detail-actions">
                <button
                  onClick={() => navigate('/announcements')}
                  className="back-button"
                >
                  목록으로
                </button>
                {isAdmin && (
                  <>
                    <button
                      onClick={() => navigate(`/announcements/edit/${id}`)}
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
