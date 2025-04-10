import React, { useState, useEffect } from 'react';
import { useParams, useNavigate } from 'react-router-dom';
import { api } from '../../utils/api';
import { useAuthStore } from '../../stores/authStore';
import { useToast } from '../../hooks/useToast';
import Layout from '../Layout/Layout';

const InventoryHistory = () => {
  const { itemId } = useParams();
  const navigate = useNavigate();
  const { token } = useAuthStore();
  const { error: showError } = useToast();
  const [history, setHistory] = useState([]);
  const [item, setItem] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchItemHistory = async () => {
      if (!token) return;
      
      setLoading(true);
      try {
        // 아이템 상세 정보 가져오기
        const itemResponse = await api.get(`/inventory/${itemId}`, {
          headers: { Authorization: `Bearer ${token}` }
        });
        setItem(itemResponse.data);
        
        // 아이템 이력 가져오기
        const historyResponse = await api.get(`/inventory/${itemId}/history`, {
          headers: { Authorization: `Bearer ${token}` }
        });
        setHistory(historyResponse.data);
      } catch (err) {
        console.error('재고 이력 조회 실패:', err);
        showError('재고 이력을 불러오는데 실패했습니다.');
      } finally {
        setLoading(false);
      }
    };
    
    fetchItemHistory();
  }, [itemId, token, showError]);

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
        <div className="p-6 text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500 mx-auto"></div>
          <p className="mt-4">재고 이력을 불러오는 중...</p>
        </div>
      </Layout>
    );
  }

  return (
    <Layout>
      <div className="p-6">
        <div className="flex justify-between items-center mb-6">
          <h1 className="text-2xl font-bold">재고 이력</h1>
          <button
            onClick={handleBackClick}
            className="bg-gray-200 hover:bg-gray-300 text-gray-800 font-bold py-2 px-4 rounded"
          >
            목록으로 돌아가기
          </button>
        </div>
        
        {item && (
          <div className="bg-white shadow-md rounded-lg p-6 mb-6">
            <h2 className="text-xl font-semibold mb-4">{item.item_name} 상세 정보</h2>
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
              <div className="border p-4 rounded-lg">
                <p className="text-gray-600 text-sm">현재 수량</p>
                <p className="text-2xl font-bold">{item.quantity}</p>
              </div>
              <div className="border p-4 rounded-lg">
                <p className="text-gray-600 text-sm">창고 수량</p>
                <p className="text-2xl font-bold">{item.warehouse_quantity}</p>
              </div>
              <div className="border p-4 rounded-lg">
                <p className="text-gray-600 text-sm">최소 기준 수량</p>
                <p className="text-2xl font-bold">{item.min_threshold}</p>
              </div>
            </div>
          </div>
        )}
        
        <div className="bg-white shadow-md rounded-lg overflow-hidden">
          <h2 className="text-xl font-semibold p-6 bg-gray-50 border-b">수량 변경 이력</h2>
          
          {history.length > 0 ? (
            <div className="overflow-x-auto">
              <table className="min-w-full divide-y divide-gray-200">
                <thead className="bg-gray-50">
                  <tr>
                    <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">날짜/시간</th>
                    <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">변경량</th>
                    <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">변경 후 수량</th>
                    <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">작업자</th>
                    <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">비고</th>
                  </tr>
                </thead>
                <tbody className="bg-white divide-y divide-gray-200">
                  {history.map((record, index) => (
                    <tr key={index} className="hover:bg-gray-50">
                      <td className="px-6 py-4 whitespace-nowrap">{formatDate(record.timestamp)}</td>
                      <td className={`px-6 py-4 whitespace-nowrap font-medium ${record.change_amount > 0 ? 'text-green-600' : 'text-red-600'}`}>
                        {record.change_amount > 0 ? `+${record.change_amount}` : record.change_amount}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap">{record.quantity_after}</td>
                      <td className="px-6 py-4 whitespace-nowrap">{record.user || '시스템'}</td>
                      <td className="px-6 py-4">{record.note || '-'}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          ) : (
            <div className="p-6 text-center text-gray-500">
              이력 데이터가 없습니다.
            </div>
          )}
        </div>
      </div>
    </Layout>
  );
};

export default InventoryHistory;
