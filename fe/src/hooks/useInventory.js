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
      // null이나 undefined를 방지하기 위해 데이터가 없으면 빈 배열로 설정
      setInventory(data || []);
      return data;
    } catch (err) {
      const errorMessage = err.message || '재고 데이터를 불러오는데 실패했습니다.';
      setError(errorMessage);
      showError(errorMessage);
      return [];
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
     
      // inventory가 배열인지 확인 후 안전하게 업데이트
      setInventory(prev => {
        // prev가 배열이 아니면 빈 배열로 초기화
        const safeInventory = Array.isArray(prev) ? prev : [];
        return safeInventory.map(item =>
          item.id === itemId ? updatedItem : item
        );
      });
     
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
     
      // inventory가 배열인지 확인 후 안전하게 업데이트
      setInventory(prev => {
        // prev가 배열이 아니면 빈 배열로 초기화
        const safeInventory = Array.isArray(prev) ? prev : [];
        return [...safeInventory, newItem];
      });
     
      return newItem;
    } catch (err) {
      const errorMessage = err.message || '새 재고 아이템 추가에 실패했습니다.';
      setError(errorMessage);
      throw err;
    } finally {
      setLoading(false);
    }
  }, [setInventory, setLoading, setError]);
  
  // 추가: 창고 수량 조정 (별도 API가 있을 경우)
  const adjustWarehouseInventory = useCallback(async (itemId, amount) => {
    setLoading(true);
    setError(null);
    try {
      // 실제 API 구현에 따라 수정 필요
      const updatedItem = await inventoryService.adjustWarehouseInventory(itemId, amount);
      
      setInventory(prev => {
        const safeInventory = Array.isArray(prev) ? prev : [];
        return safeInventory.map(item =>
          item.id === itemId ? updatedItem : item
        );
      });
      
      return updatedItem;
    } catch (err) {
      const errorMessage = err.message || '창고 수량 조정에 실패했습니다.';
      setError(errorMessage);
      throw err;
    } finally {
      setLoading(false);
    }
  }, [setInventory, setLoading, setError]);
  
  return {
    // 항상 안전한 배열을 반환하도록 보장
    inventory: Array.isArray(inventory) ? inventory : [],
    loading,
    error,
    fetchInventory,
    adjustInventory,
    addInventoryItem,
    adjustWarehouseInventory
  };
};