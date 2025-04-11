import { api } from '../utils/api';

/**
 * 재고 관리 관련 API 서비스
 */
export const inventoryService = {
  /**
   * 모든 재고 아이템 조회
   * @returns {Promise<Array>} 재고 아이템 목록
   */
  getInventory: async () => {
    try {
      const response = await api.get('/inventory');
      // 속성명 변환 처리 (API 응답 형식에 맞춤)
      return response.data.map(item => ({
        id: item.id,
        item_name: item.itemName,
        quantity: item.quantity,
        warehouse_quantity: item.warehouseQuantity,
        min_threshold: item.minThreshold,
        shelf_code: item.shelfCode // 선반 코드 추가
      }));
    } catch (error) {
      console.error('재고 데이터 조회 실패:', error);
      throw new Error(error.response?.data?.message || '재고 데이터를 불러오는데 실패했습니다.');
    }
  },

  /**
   * 재고 수량 조정
   * @param {number} itemId - 아이템 ID
   * @param {number} amount - 조정할 수량 (양수: 증가, 음수: 감소)
   * @returns {Promise<Object>} 업데이트된 재고 아이템 정보
   */
  adjustInventory: async (itemId, amount) => {
    try {
      const response = await api.post(`/inventory/${itemId}/adjust?amount=${amount}`);
      // 응답 데이터 속성명 변환
      const item = response.data;
      return {
        id: item.id,
        item_name: item.item_name || item.itemName,
        quantity: item.quantity,
        warehouse_quantity: item.warehouse_quantity || item.warehouseQuantity,
        min_threshold: item.min_threshold || item.minThreshold,
        shelf_code: item.shelf_code || item.shelfCode // 선반 코드 추가
      };
    } catch (error) {
      console.error('재고 수량 조정 실패:', error);
      throw new Error(error.response?.data?.message || '재고 수량 조정에 실패했습니다.');
    }
  },

  /**
   * 새 재고 아이템 추가
   * @param {Object} itemData - 새 아이템 데이터
   * @param {string} itemData.item_name - 아이템 이름
   * @param {number} itemData.quantity - 초기 수량
   * @param {number} itemData.warehouse_quantity - 창고 수량
   * @param {number} itemData.min_threshold - 최소 재고 수량
   * @returns {Promise<Object>} 생성된 재고 아이템 정보
   */
  addInventoryItem: async (itemData) => {
    try {
      // API 요청 형식에 맞게 데이터 변환
      const requestData = {
        itemName: itemData.item_name,
        quantity: itemData.quantity,
        warehouseQuantity: itemData.warehouse_quantity,
        minThreshold: itemData.min_threshold,
        shelfCode: itemData.shelf_code // 선반 코드 추가
      };
      
      const response = await api.post('/inventory', requestData);
      
      // 응답 데이터 속성명 변환
      const item = response.data;
      return {
        id: item.id,
        item_name: item.item_name || item.itemName,
        quantity: item.quantity,
        warehouse_quantity: item.warehouse_quantity || item.warehouseQuantity,
        min_threshold: item.min_threshold || item.minThreshold,
        shelf_code: item.shelf_code || item.shelfCode // 선반 코드 추가
      };
    } catch (error) {
      console.error('재고 아이템 추가 실패:', error);
      throw new Error(error.response?.data?.message || '새 재고 아이템 추가에 실패했습니다.');
    }
  }
};
