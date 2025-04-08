import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { useNavigate } from 'react-router-dom';
import './AnnouncementList.css';

function AnnouncementList() {
  const navigate = useNavigate();
  const [announcements, setAnnouncements] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [currentPage, setCurrentPage] = useState(1);
  const [isAdmin, setIsAdmin] = useState(false);
  const [searchKeyword, setSearchKeyword] = useState('');
  const [searchType, setSearchType] = useState('title'); // 'title' 또는 'content'
  const itemsPerPage = 15;

  useEffect(() => {
    fetchAnnouncements();
    checkUserRole();
  }, []);

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

  const fetchAnnouncements = async () => {
    try {
      const accessToken = localStorage.getItem('accessToken');
      if (!accessToken) {
        navigate('/');
        return;
      }

      const response = await axios.get(
        'http://localhost:8080/api/boards/announcements',
        {
          headers: {
            Authorization: `Bearer ${accessToken}`,
          },
        }
      );

      if (Array.isArray(response.data)) {
        const formattedAnnouncements = response.data.map((announcement) => ({
          id: announcement.id,
          title: announcement.title,
          content: announcement.content,
          writer: announcement.writer,
          viewCount: announcement.view_count || 0,
          createdAt: announcement.created_at,
          updatedAt: announcement.updated_at,
          expirationDate: announcement.expiration_date,
        }));
        setAnnouncements(formattedAnnouncements);
      }
    } catch (error) {
      if (error.response && error.response.status === 403) {
        localStorage.removeItem('accessToken');
        navigate('/');
      }
    } finally {
      setIsLoading(false);
    }
  };

  const handleSearch = () => {
    if (!searchKeyword.trim()) {
      fetchAnnouncements();
      return;
    }

    const filteredAnnouncements = announcements.filter((announcement) => {
      if (searchType === 'title') {
        return announcement.title
          .toLowerCase()
          .includes(searchKeyword.toLowerCase());
      } else if (searchType === 'content') {
        return announcement.content
          .toLowerCase()
          .includes(searchKeyword.toLowerCase());
      }
      return false;
    });

    setAnnouncements(filteredAnnouncements);
    setCurrentPage(1); // 검색 결과가 변경되면 첫 페이지로 이동
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter') {
      handleSearch();
    }
  };

  const handleReset = () => {
    setSearchKeyword('');
    setSearchType('title');
    fetchAnnouncements();
  };

  const totalPages = Math.ceil(announcements.length / itemsPerPage);
  const startIndex = (currentPage - 1) * itemsPerPage;
  const endIndex = startIndex + itemsPerPage;
  const currentAnnouncements = announcements.slice(startIndex, endIndex);

  const handlePageChange = (page) => {
    setCurrentPage(page);
  };

  if (isLoading) {
    return (
      <div className="loading-container">
        <div className="loading-spinner"></div>
      </div>
    );
  }

  return (
    <div className="announcement-page">
      <div className="announcement-container">
        {/* 페이지 헤더 */}
        <div className="announcement-header">
          <h1 className="announcement-title">
            <span className="announcement-title-gradient">물류 공지사항</span>
          </h1>
        </div>

        {/* 검색 기능 */}
        <div className="search-container">
          <div className="search-box">
            <select
              value={searchType}
              onChange={(e) => setSearchType(e.target.value)}
              className="search-select"
            >
              <option value="title">제목</option>
              <option value="content">내용</option>
            </select>
            <input
              type="text"
              placeholder="검색어를 입력하세요"
              value={searchKeyword}
              onChange={(e) => setSearchKeyword(e.target.value)}
              onKeyPress={handleKeyPress}
              className="search-input"
            />
            <button onClick={handleSearch} className="search-button">
              검색
            </button>
            <button onClick={handleReset} className="reset-button">
              초기화
            </button>
          </div>
        </div>

        {/* 테이블과 페이지네이션 컨테이너 */}
        <div className="content-container">
          {/* 공지사항 테이블 */}
          <div className="announcement-table-container">
            <div className="table-header">
              <h2 className="table-title">전체 물류 공지사항</h2>
              {isAdmin && (
                <button
                  onClick={() => navigate('/announcements/write')}
                  className="write-button"
                >
                  <svg
                    className="h-5 w-5 mr-2"
                    fill="none"
                    stroke="currentColor"
                    viewBox="0 0 24 24"
                  >
                    <path
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeWidth="2"
                      d="M12 4v16m8-8H4"
                    />
                  </svg>
                  새 물류 공지 작성
                </button>
              )}
            </div>

            <div className="table-content">
              <table className="table">
                <thead>
                  <tr>
                    <th>제목</th>
                    <th style={{ width: '160px' }}>작성자</th>
                    <th style={{ width: '160px' }}>작성일</th>
                    <th style={{ width: '128px' }}>조회수</th>
                  </tr>
                </thead>
                <tbody>
                  {currentAnnouncements.length === 0 ? (
                    <tr>
                      <td colSpan="4" className="empty-message">
                        {searchKeyword
                          ? '검색 결과가 없습니다.'
                          : '등록된 물류 공지사항이 없습니다.'}
                      </td>
                    </tr>
                  ) : (
                    currentAnnouncements.map((announcement) => (
                      <tr
                        key={announcement.id}
                        onClick={() =>
                          navigate(`/announcements/${announcement.id}`)
                        }
                        className="table-row"
                      >
                        <td className="font-medium text-gray-900 hover:text-amber-600">
                          {announcement.title}
                        </td>
                        <td className="text-gray-500">{announcement.writer}</td>
                        <td className="text-gray-500">
                          {new Date(
                            announcement.createdAt
                          ).toLocaleDateString()}
                        </td>
                        <td className="text-gray-500">
                          <div className="view-count">
                            <svg
                              fill="none"
                              stroke="currentColor"
                              viewBox="0 0 24 24"
                            >
                              <path
                                strokeLinecap="round"
                                strokeLinejoin="round"
                                strokeWidth="2"
                                d="M15 12a3 3 0 11-6 0 3 3 0 016 0z"
                              />
                              <path
                                strokeLinecap="round"
                                strokeLinejoin="round"
                                strokeWidth="2"
                                d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z"
                              />
                            </svg>
                            {announcement.viewCount}
                          </div>
                        </td>
                      </tr>
                    ))
                  )}
                </tbody>
              </table>
            </div>
          </div>

          {/* 페이지네이션 */}
          {announcements.length > 0 && (
            <div className="pagination-container">
              <nav className="pagination-nav" aria-label="Pagination">
                <button
                  onClick={() => handlePageChange(currentPage - 1)}
                  disabled={currentPage === 1}
                  className="pagination-button"
                >
                  <span className="sr-only">이전 페이지</span>
                  <svg
                    className="h-5 w-5"
                    fill="none"
                    stroke="currentColor"
                    viewBox="0 0 24 24"
                  >
                    <path
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeWidth="2"
                      d="M15 19l-7-7 7-7"
                    />
                  </svg>
                </button>
                {Array.from({ length: totalPages }, (_, i) => i + 1).map(
                  (page) => (
                    <button
                      key={page}
                      onClick={() => handlePageChange(page)}
                      className={`page-button ${
                        currentPage === page ? 'active' : ''
                      }`}
                    >
                      {page}
                    </button>
                  )
                )}
                <button
                  onClick={() => handlePageChange(currentPage + 1)}
                  disabled={currentPage === totalPages}
                  className="pagination-button"
                >
                  <span className="sr-only">다음 페이지</span>
                  <svg
                    className="h-5 w-5"
                    fill="none"
                    stroke="currentColor"
                    viewBox="0 0 24 24"
                  >
                    <path
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeWidth="2"
                      d="M9 5l7 7-7 7"
                    />
                  </svg>
                </button>
              </nav>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

export default AnnouncementList;
