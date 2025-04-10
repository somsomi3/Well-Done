import { useCallback } from 'react';
import { useInventoryStore } from '../stores/inventoryStore';
import { inventoryService } from '../services/inventoryService';
import { useToast } from './useToast';

export const useInventory = () => {
  const { 
    inventory, 
    setInventory, 
    setLoading, 
    setError, 
    loading, 
    error 
  } = useInventoryStore();
  const { error: showError } = useToast();

  // 모든 재고 아이템 가져오기
  const fetchInventory = useCallback(async () => {
    setLoading(true);
    setError(null);

    try {
      const data = await inventoryService.getInventory();
      setInventory(data);
      return data;
    } catch (err) {
      const errorMessage = err.message || '재고 데이터를 불러오는데 실패했습니다.';
      setError(errorMessage);
      showError(errorMessage);
      return null;
    } finally {
      setLoading(false);
    }
  }, [setInventory, setLoading, setError, showError]);

  // 재고 수량 조정
  const adjustInventory = useCallback(async (itemId, amount) => {
    setLoading(true);
    setError(null);

    try {
      const updatedItem = await inventoryService.adjustInventory(itemId, amount);
      
      // 상태 업데이트
      setInventory(prev => 
        prev.map(item => 
          item.id === itemId ? updatedItem : item
        )
      );
      
      return updatedItem;
    } catch (err) {
      const errorMessage = err.message || '재고 수량 조정에 실패했습니다.';
      setError(errorMessage);
      throw err;
    } finally {
      setLoading(false);
    }
  }, [setInventory, setLoading, setError]);

  // 새 재고 아이템 추가
  const addInventoryItem = useCallback(async (itemData) => {
    setLoading(true);
    setError(null);

    try {
      const newItem = await inventoryService.addInventoryItem(itemData);
      
      // 상태 업데이트
      setInventory(prev => [...prev, newItem]);
      
      return newItem;
    } catch (err) {
      const errorMessage = err.message || '새 재고 아이템 추가에 실패했습니다.';
      setError(errorMessage);
      throw err;
    } finally {
      setLoading(false);
    }
  }, [setInventory, setLoading, setError]);

  return {
    inventory,
    loading,
    error,
    fetchInventory,
    adjustInventory,
    addInventoryItem
  };
};
