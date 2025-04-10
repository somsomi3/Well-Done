import React, { useState, useEffect } from 'react';
import { useParams, useNavigate } from 'react-router-dom';
import Layout from '../components/Layout/Layout';
import { api } from '../utils/api';
import { useAuthStore } from '../stores/authStore';
import { useToast } from '../hooks/useToast';
import '../styles/InventoryStyles.css';

function InventoryDetailPage() {
  const { itemId } = useParams();
  const navigate = useNavigate();
  const { token } = useAuthStore();
  const { success, error: showError } = useToast();
  const [item, setItem] = useState(null);
  const [history, setHistory] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchItemDetails = async () => {
      if (!token || !itemId) return;
      
      setLoading(true);
      try {
        // 아이템 상세 정보 가져오기
        const itemResponse = await api.get(`/inventory/${itemId}`);
        setItem(itemResponse.data);
        
        // 아이템 이력 가져오기
        const historyResponse = await api.get(`/inventory/${itemId}/history`);
        setHistory(historyResponse.data);
        
        success('재고 상세 정보를 불러왔습니다.');
      } catch (err) {
        console.error('재고 상세 정보 조회 실패:', err);
        showError('재고 상세 정보를 불러오는데 실패했습니다.');
      } finally {
        setLoading(false);
      }
    };
    
    fetchItemDetails();
  }, [itemId, token, success, showError]);

  const formatDate = (dateString) => {
    const date = new Date(dateString);
    return new Intl.DateTimeFormat('ko-KR', {
      year: 'numeric',
      month: '2-digit',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit'
    }).format(date);
  };

  const handleBackClick = () => {
    navigate('/log');
  };

  if (loading) {
    return (
      <Layout>
        <div className="inventory-loading">
          <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
          <p className="mt-4">재고 상세 정보를 불러오는 중...</p>
        </div>
      </Layout>
    );
  }

  return (
    <Layout>
      <div className="inventory-page">
        <div className="inventory-header-container">
          <h1 className="inventory-title">재고 상세 정보</h1>
          <button
            onClick={handleBackClick}
            className="back-button"
          >
            목록으로 돌아가기
          </button>
        </div>
        
        {item && (
          <div className="inventory-detail-card">
            <h2 className="inventory-detail-title">{item.item_name} 상세 정보</h2>
            <div className="inventory-detail-grid">
              <div className="inventory-detail-item">
                <p className="inventory-detail-label">현재 수량</p>
                <p className="inventory-detail-value">{item.quantity}</p>
              </div>
              <div className="inventory-detail-item">
                <p className="inventory-detail-label">창고 수량</p>
                <p className="inventory-detail-value">{item.warehouse_quantity}</p>
              </div>
              <div className="inventory-detail-item">
                <p className="inventory-detail-label">최소 기준 수량</p>
                <p className="inventory-detail-value">{item.min_threshold}</p>
              </div>
              {item.shelf_code && (
                <div className="inventory-detail-item">
                  <p className="inventory-detail-label">선반 코드</p>
                  <p className="inventory-detail-value">{item.shelf_code}</p>
                </div>
              )}
              {item.coordinate && (
                <div className="inventory-detail-item">
                  <p className="inventory-detail-label">좌표</p>
                  <p className="inventory-detail-value">
                    X: {item.coordinate.x}, Y: {item.coordinate.y}, 각도: {item.coordinate.angle}°
                  </p>
                </div>
              )}
            </div>
          </div>
        )}
        
        <div className="inventory-history-container">
          <h2 className="inventory-history-title">수량 변경 이력</h2>
          
          {history.length > 0 ? (
            <div className="inventory-table-container">
              <table className="inventory-table">
                <thead>
                  <tr>
                    <th>날짜/시간</th>
                    <th>변경량</th>
                    <th>변경 후 수량</th>
                    <th>작업자</th>
                    <th>비고</th>
                  </tr>
                </thead>
                <tbody>
                  {history.map((record, index) => (
                    <tr key={index} className="inventory-history-item">
                      <td>{formatDate(record.timestamp)}</td>
                      <td className={`change-amount ${record.change_amount > 0 ? 'positive' : 'negative'}`}>
                        {record.change_amount > 0 ? `+${record.change_amount}` : record.change_amount}
                      </td>
                      <td>{record.quantity_after}</td>
                      <td>{record.user || '시스템'}</td>
                      <td>{record.note || '-'}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          ) : (
            <div className="inventory-empty">
              이력 데이터가 없습니다.
            </div>
          )}
        </div>
      </div>
    </Layout>
  );
}

export default InventoryDetailPage;
