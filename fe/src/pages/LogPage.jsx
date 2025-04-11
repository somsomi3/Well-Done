import React, { useEffect, useState } from 'react';
import Layout from '../components/Layout/Layout';
import { useAuthStore } from '../stores/authStore';
import api from '../utils/api';

function LogPage() {
  const [inventoryLogs, setInventoryLogs] = useState([]);
  const [inventoryList, setInventoryList] = useState([]);
  const [newItem, setNewItem] = useState({
    item_name: '',
    quantity: '',
    min_threshold: '',
    angle: ''
  });
  const [alertMsgMap, setAlertMsgMap] = useState({});
  const [loading, setLoading] = useState(true);
  const { token } = useAuthStore();

  // API 1: 전체 재고 목록 조회
  const fetchInventoryList = async () => {
    try {
      const res = await api.get('/inventory', {
        headers: { Authorization: `Bearer ${token}` }
      });
      setInventoryList(res.data);
    } catch (err) {
      console.error('재고 목록 조회 실패:', err);
      alert('재고 정보를 불러오지 못했습니다.');
    }
  };

  // API 2: 재고 가감 및 로봇 명령
  const updateStock = async (itemId, amount) => {
    try {
      const res = await api.post(`/inventory/${itemId}/adjust?amount=${amount}`, null, {
        headers: { Authorization: `Bearer ${token}` }
      });

      const updated = res.data;
      setInventoryList(prev => prev.map(item => (item.id === updated.id ? updated : item)));

      setAlertMsgMap(prev => ({
        ...prev,
        [itemId]: updated.quantity <= updated.min_threshold
          ? `⚠️ ${updated.item_name} 재고 부족! (${updated.quantity})`
          : ''
      }));

      // API 3: 재고가 0이면 로봇 명령 전송
      if (updated.quantity === 0) {
        await api.post('/robot/pick-place', {
          from: { x: 1.5, y: 2.5, angle: 1.57 },
          to: { x: 8.0, y: 3.2, angle: 0.0 },
          product_id: String(updated.id),
          display_spot: 7
        }, { headers: { Authorization: `Bearer ${token}` } });

        alert('🤖 로봇 명령이 전송되었습니다!');
      }

      fetchInventoryLogs(); // 이력 갱신
    } catch (err) {
      console.error('재고 조정 실패:', err);
      alert('재고 조정 중 오류가 발생했습니다.');
    }
  };

  // API 4: 새 아이템 추가
  const handleAddInventory = async (e) => {
    e.preventDefault();
    try {
      await api.post('/inventory', {
        ...newItem,
        quantity: parseInt(newItem.quantity),
        min_threshold: parseInt(newItem.min_threshold),
        position: { x: -49.30, y: -63.09, angle: -90 },
        display_position: { x: -49.83, y: -62.60, angle: -90 }
      }, { headers: { Authorization: `Bearer ${token}` } });

      alert('✅ 재고가 등록되었습니다.');
      setNewItem({ item_name: '', quantity: '', min_threshold: '', angle: '' });
      fetchInventoryList();
      fetchInventoryLogs();
    } catch (err) {
      console.error('재고 등록 실패:', err);
      alert('재고 등록에 실패했습니다.');
    }
  };

  // 재고 이력 조회
  const fetchInventoryLogs = async () => {
    try {
      const response = await api.get('/inventory/logs', {
        headers: { Authorization: `Bearer ${token}` }
      });
      setInventoryLogs(response.data);
    } catch (error) {
      console.error('이력 조회 실패:', error);
      alert('이력 정보를 불러오지 못했습니다.');
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    if (token) {
      fetchInventoryList();
      fetchInventoryLogs();
    }
  }, [token]);

  if (loading) return <div>🔄 로딩 중...</div>;

  return (
    <Layout>
      <div className="p-6">
        <h1 className="text-2xl font-bold mb-6">📦 재고 관리 및 이력</h1>

        {/* 새 아이템 등록 폼 */}
        <form onSubmit={handleAddInventory} className="mb-8 p-4 bg-gray-50 rounded-lg">
          <div className="grid grid-cols-2 gap-4 mb-4">
            <input name="item_name" placeholder="상품명" value={newItem.item_name}
              onChange={(e) => setNewItem(prev => ({ ...prev, [e.target.name]: e.target.value }))}
              className="p-2 border rounded" required />
            <input type="number" name="quantity" placeholder="초기 수량"
              value={newItem.quantity} onChange={(e) => setNewItem(prev => ({ ...prev, [e.target.name]: e.target.value }))}
              className="p-2 border rounded" required />
            <input type="number" name="min_threshold" placeholder="최소 수량"
              value={newItem.min_threshold} onChange={(e) => setNewItem(prev => ({ ...prev, [e.target.name]: e.target.value }))}
              className="p-2 border rounded" required />
            <input type="number" name="angle" placeholder="창고 각도" step="0.1"
              value={newItem.angle} onChange={(e) => setNewItem(prev => ({ ...prev, [e.target.name]: e.target.value }))}
              className="p-2 border rounded" required />
          </div>
          <button type="submit" className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600">
            ➕ 새 상품 등록
          </button>
        </form>

        {/* 실시간 재고 현황 */}
        <div className="mb-8">
          <h2 className="text-xl font-semibold mb-4">🔍 실시간 재고 현황</h2>
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
            {inventoryList.map(item => (
              <div key={item.id} className="p-4 bg-white rounded-lg shadow-md">
                <h3 className="font-bold mb-2">{item.item_name}</h3>
                <p>현재 수량: {item.quantity}</p>
                <p>안전 재고 기준 수량: {item.min_threshold}</p>
                <div className="flex gap-2 mt-3">
                  <button onClick={() => updateStock(item.id, -25)}
                    className="px-3 py-1 bg-red-100 text-red-600 rounded hover:bg-red-200">
                    ➖ 감소
                  </button>
                  <button onClick={() => updateStock(item.id, 25)}
                    className="px-3 py-1 bg-green-100 text-green-600 rounded hover:bg-green-200">
                    ➕ 증가
                  </button>
                </div>
                {alertMsgMap[item.id] && (
                  <div className="mt-2 p-2 bg-yellow-100 text-yellow-700 rounded-md">
                    ⚠️ 재고 부족 경고
                  </div>
                )}
              </div>
            ))}
          </div>
        </div>

        {/* 재고 변동 이력 */}
        <div className="bg-white rounded-lg shadow-md p-4">
          <h2 className="text-xl font-semibold mb-4">📜 재고 변동 이력</h2>
          <div className="overflow-x-auto">
            <table className="w-full">
              <thead className="bg-gray-50">
                <tr>
                  <th className="p-2 text-left">시간</th>
                  <th className="p-2 text-left">상품명</th>
                  <th className="p-2 text-left">변동량</th>
                  <th className="p-2 text-left">최종 수량</th>
                </tr>
              </thead>
              <tbody>
                {inventoryLogs.map(log => (
                  <tr key={log.id} className="border-b hover:bg-gray-50">
                    <td className="p-2">{new Date(log.timestamp).toLocaleString()}</td>
                    <td className="p-2">{log.item_name}</td>
                    <td className={`p-2 ${log.change_amount > 0 ? 'text-green-600' : 'text-red-600'}`}>
                      {log.change_amount > 0 ? `+${log.change_amount}` : log.change_amount}
                    </td>
                    <td className="p-2">{log.current_quantity}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default LogPage;
