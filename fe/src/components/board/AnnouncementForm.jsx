import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import axios from 'axios';
import Layout from '../Layout/Layout';
import './AnnouncementForm.css'; // CSS 파일 import (별도로 생성 필요)

function AnnouncementForm() {
  const navigate = useNavigate();
  const [title, setTitle] = useState('');
  const [content, setContent] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (isSubmitting) return;

    try {
      setIsSubmitting(true);
      const response = await axios.post('/api/announcements', {
        title,
        content,
      });

      if (response.status === 201) {
        navigate('/announcements');
      }
    } catch (error) {
      console.error('공지사항 작성 중 오류 발생:', error);
      alert('공지사항 작성에 실패했습니다. 다시 시도해주세요.');
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Layout>
      <div className="announcement-form-page">
        <div className="announcement-form-container">
          <div className="announcement-form-header">
            <h1 className="announcement-form-title">새 물류 공지 작성</h1>
            <p className="announcement-form-subtitle">
              물류 관련 중요 공지사항을 작성하는 공간입니다
            </p>
          </div>
          <form onSubmit={handleSubmit} className="announcement-form">
            <div className="form-group">
              <label htmlFor="title" className="form-label">
                제목 <span className="required">*</span>
              </label>
              <input
                type="text"
                id="title"
                value={title}
                onChange={(e) => setTitle(e.target.value)}
                className="form-input"
                placeholder="공지사항 제목을 입력하세요"
                required
              />
            </div>
            <div className="form-group">
              <label htmlFor="content" className="form-label">
                내용 <span className="required">*</span>
              </label>
              <textarea
                id="content"
                value={content}
                onChange={(e) => setContent(e.target.value)}
                rows="15"
                className="form-textarea"
                placeholder="공지사항 내용을 작성하세요..."
                required
              />
            </div>
            <div className="form-actions">
              <button
                type="button"
                onClick={() => navigate('/announcements')}
                className="cancel-button"
              >
                취소
              </button>
              <button
                type="submit"
                disabled={isSubmitting}
                className="submit-button"
              >
                {isSubmitting ? (
                  <>
                    <span className="spinner"></span>
                    작성 중...
                  </>
                ) : (
                  '공지사항 등록'
                )}
              </button>
            </div>
          </form>
        </div>
      </div>
    </Layout>
  );
}

export default AnnouncementForm;
