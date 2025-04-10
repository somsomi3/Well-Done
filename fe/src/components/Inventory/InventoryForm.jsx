import React, { useState } from 'react';
import { useInventory } from '../../hooks/useInventory';
import { useToast } from '../../hooks/useToast';

const InventoryForm = ({ onClose }) => {
  const { addInventoryItem } = useInventory();
  const { success, error } = useToast();
  const [formData, setFormData] = useState({
    item_name: '',
    quantity: 0,
    warehouse_quantity: 0,
    min_threshold: 0,
    shelf_code: '' // 선반 코드 추가
  });
  const [isSubmitting, setIsSubmitting] = useState(false);

  const handleChange = (e) => {
    const { name, value } = e.target;
    
    // 숫자 필드는 숫자로 변환
    if (['quantity', 'warehouse_quantity', 'min_threshold'].includes(name)) {
      const numValue = value === '' ? '' : parseInt(value, 10);
      if (isNaN(numValue)) return;
      setFormData(prev => ({ ...prev, [name]: numValue }));
    } else {
      setFormData(prev => ({ ...prev, [name]: value }));
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    
    if (!formData.item_name) {
      error('제품명을 입력해주세요.');
      return;
    }

    // 음수 값 검증
    if (formData.quantity < 0 || formData.warehouse_quantity < 0 || formData.min_threshold < 0) {
      error('수량은 0 이상이어야 합니다.');
      return;
    }

    setIsSubmitting(true);

    try {
      await addInventoryItem(formData);
      success(`${formData.item_name} 아이템이 추가되었습니다.`);
      onClose();
    } catch (err) {
      error(`아이템 추가 중 오류가 발생했습니다: ${err.message}`);
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className="inventory-form-container">
      <h2 className="inventory-form-title">새 재고 아이템 추가</h2>
      <form onSubmit={handleSubmit} className="inventory-form">
        <div className="form-group">
          <label htmlFor="item_name">제품명 *</label>
          <input
            type="text"
            id="item_name"
            name="item_name"
            value={formData.item_name}
            onChange={handleChange}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="shelf_code">선반 코드</label>
          <input
            type="text"
            id="shelf_code"
            name="shelf_code"
            value={formData.shelf_code}
            onChange={handleChange}
            placeholder="예: A1, B2 등"
          />
        </div>

        <div className="form-group">
          <label htmlFor="quantity">현재 수량</label>
          <input
            type="number"
            id="quantity"
            name="quantity"
            value={formData.quantity}
            onChange={handleChange}
            min="0"
          />
        </div>

        <div className="form-group">
          <label htmlFor="warehouse_quantity">창고 수량</label>
          <input
            type="number"
            id="warehouse_quantity"
            name="warehouse_quantity"
            value={formData.warehouse_quantity}
            onChange={handleChange}
            min="0"
          />
        </div>

        <div className="form-group">
          <label htmlFor="min_threshold">최소 수량</label>
          <input
            type="number"
            id="min_threshold"
            name="min_threshold"
            value={formData.min_threshold}
            onChange={handleChange}
            min="0"
          />
        </div>

        <div className="form-actions">
          <button 
            type="submit" 
            className="submit-button"
            disabled={isSubmitting}
          >
            {isSubmitting ? '처리 중...' : '추가'}
          </button>
          <button 
            type="button" 
            className="cancel-button"
            onClick={onClose}
          >
            취소
          </button>
        </div>
      </form>
    </div>
  );
};

export default InventoryForm;
