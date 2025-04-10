import { create } from 'zustand';

/**
 * 재고 관리 상태 저장소
 */
export const useInventoryStore = create((set, get) => ({
  // 상태
  inventory: [],
  loading: false,
  error: null,

  // 재고 목록 설정
  setInventory: (inventory) => {
    console.log('재고 데이터 상태 업데이트:', inventory);
    set({ inventory });
  },

  // 로딩 상태 설정
  setLoading: (loading) => set({ loading }),

  // 에러 상태 설정
  setError: (error) => set({ error }),

  // 특정 아이템 업데이트
  updateInventoryItem: (itemId, updatedData) => {
    set((state) => ({
      inventory: state.inventory.map((item) => 
        item.id === itemId ? { ...item, ...updatedData } : item
      )
    }));
  },

  // 새 아이템 추가
  addInventoryItem: (newItem) => {
    set((state) => ({
      inventory: [...state.inventory, newItem]
    }));
  },

  // 아이템 제거 (필요한 경우)
  removeInventoryItem: (itemId) => {
    set((state) => ({
      inventory: state.inventory.filter((item) => item.id !== itemId)
    }));
  },

  // 재고 데이터 초기화
  clearInventory: () => {
    set({ inventory: [], error: null });
  }
}));
