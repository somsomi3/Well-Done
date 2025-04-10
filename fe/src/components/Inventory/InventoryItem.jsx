import React, { useState } from 'react';
import { useInventory } from '../../hooks/useInventory';
import { useToast } from '../../hooks/useToast';

const InventoryItem = ({ item }) => {
  const { adjustInventory } = useInventory();
  const { success, error } = useToast();
  const [adjustAmount, setAdjustAmount] = useState('');
  const [isAdjusting, setIsAdjusting] = useState(false);

  const handleAdjustClick = () => {
    setIsAdjusting(true);
  };

  const handleCancelAdjust = () => {
    setIsAdjusting(false);
    setAdjustAmount('');
  };

  const handleAmountChange = (e) => {
    // 숫자만 입력 가능하도록 처리 (음수 허용)
    const value = e.target.value;
    if (value === '' || /^-?\d+$/.test(value)) {
      setAdjustAmount(value);
    }
  };

  const handleSubmitAdjust = async () => {
    if (adjustAmount === '') {
      error('수량을 입력해주세요.');
      return;
    }

    try {
      await adjustInventory(item.id, parseInt(adjustAmount, 10));
      success(`${item.item_name}의 수량이 ${adjustAmount > 0 ? '증가' : '감소'}되었습니다.`);
      setIsAdjusting(false);
      setAdjustAmount('');
    } catch (err) {
      error(`수량 조정 중 오류가 발생했습니다: ${err.message}`);
    }
  };

  return (
    <tr className="inventory-item">
      <td>{item.id}</td>
      <td>{item.item_name}</td>
      <td>{item.quantity}</td>
      <td>{item.warehouse_quantity}</td>
      <td>{item.min_threshold}</td>
      <td>
        {isAdjusting ? (
          <div className="adjust-controls">
            <input
              type="text"
              value={adjustAmount}
              onChange={handleAmountChange}
              placeholder="±수량"
              className="adjust-input"
            />
            <button 
              onClick={handleSubmitAdjust}
              className="adjust-button confirm"
            >
              확인
            </button>
            <button 
              onClick={handleCancelAdjust}
              className="adjust-button cancel"
            >
              취소
            </button>
          </div>
        ) : (
          <button 
            onClick={handleAdjustClick}
            className="adjust-button"
          >
            수량 조정
          </button>
        )}
      </td>
    </tr>
  );
};

export default InventoryItem;
