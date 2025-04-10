import React, { useState, useEffect } from 'react';
import { useInventory } from '../../hooks/useInventory';
import InventoryItem from './InventoryItem';
import { useToast } from '../../hooks/useToast';
import '../../styles/InventoryStyles.css';

const InventoryList = ({ onAddInventory }) => {
  const { inventory, loading, error, fetchInventory } = useInventory();
  const { error: showError } = useToast();
  const [searchTerm, setSearchTerm] = useState('');
  const [sortConfig, setSortConfig] = useState({ key: 'id', direction: 'ascending' });

  // 컴포넌트 마운트 시 데이터 로드 (이미 InventoryPage에서 호출하므로 선택적)
  useEffect(() => {
    console.log('InventoryList 마운트, 현재 데이터:', inventory);
    if (!inventory || inventory.length === 0) {
      fetchInventory();
    }
  }, [inventory, fetchInventory]);

  useEffect(() => {
    if (error) {
      showError(`재고 데이터를 불러오는 중 오류가 발생했습니다: ${error}`);
    }
  }, [error, showError]);

  const handleSearch = (e) => {
    setSearchTerm(e.target.value);
  };

  const handleSort = (key) => {
    let direction = 'ascending';
    if (sortConfig.key === key && sortConfig.direction === 'ascending') {
      direction = 'descending';
    }
    setSortConfig({ key, direction });
  };

  // 정렬 및 필터링된 재고 목록
  const sortedInventory = React.useMemo(() => {
    if (!inventory || inventory.length === 0) {
      console.log('정렬할 재고 데이터가 없습니다');
      return [];
    }
    
    console.log('정렬 전 재고 데이터:', inventory);
    
    let sortableItems = [...inventory];
    
    // 검색어로 필터링
    if (searchTerm) {
      sortableItems = sortableItems.filter(item => 
        item.item_name.toLowerCase().includes(searchTerm.toLowerCase())
      );
    }
    
    // 정렬
    sortableItems.sort((a, b) => {
      if (a[sortConfig.key] < b[sortConfig.key]) {
        return sortConfig.direction === 'ascending' ? -1 : 1;
      }
      if (a[sortConfig.key] > b[sortConfig.key]) {
        return sortConfig.direction === 'ascending' ? 1 : -1;
      }
      return 0;
    });
    
    console.log('정렬 후 재고 데이터:', sortableItems);
    return sortableItems;
  }, [inventory, searchTerm, sortConfig]);

  const getSortIndicator = (key) => {
    if (sortConfig.key !== key) return null;
    return sortConfig.direction === 'ascending' ? '↑' : '↓';
  };

  if (loading) {
    return <div className="inventory-loading">재고 데이터를 불러오는 중...</div>;
  }

  return (
    <div className="inventory-container">
      <div className="inventory-header">
        <div className="inventory-actions">
          <input
            type="text"
            placeholder="제품명으로 검색..."
            value={searchTerm}
            onChange={handleSearch}
            className="inventory-search"
          />
          <button 
            onClick={onAddInventory}
            className="add-inventory-button"
          >
            새 재고 추가
          </button>
        </div>
      </div>

      <div className="inventory-table-container">
        <table className="inventory-table">
          <thead>
            <tr>
              <th onClick={() => handleSort('id')}>
                ID {getSortIndicator('id')}
              </th>
              <th onClick={() => handleSort('item_name')}>
                제품명 {getSortIndicator('item_name')}
              </th>
              <th onClick={() => handleSort('quantity')}>
                현재 수량 {getSortIndicator('quantity')}
              </th>
              <th onClick={() => handleSort('warehouse_quantity')}>
                창고 수량 {getSortIndicator('warehouse_quantity')}
              </th>
              <th onClick={() => handleSort('min_threshold')}>
                최소 수량 {getSortIndicator('min_threshold')}
              </th>
              <th>작업</th>
            </tr>
          </thead>
          <tbody>
            {sortedInventory.length > 0 ? (
              sortedInventory.map(item => (
                <InventoryItem key={item.id} item={item} />
              ))
            ) : (
              <tr>
                <td colSpan="6" className="inventory-empty">
                  {searchTerm ? '검색 결과가 없습니다.' : '재고 데이터가 없습니다.'}
                </td>
              </tr>
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default InventoryList;
