package com.be.domain.inventory.service;

import com.be.db.entity.Inventory;
import com.be.db.entity.InventoryHistory;
import com.be.db.repository.InventoryHistoryRepository;
import com.be.domain.inventory.dto.InventoryHistoryDto;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
@RequiredArgsConstructor
public class InventoryHistoryService {

    private final InventoryHistoryRepository inventoryHistoryRepository;

    /**
     * 재고 변경 이력 저장
     * @param inventory 대상 재고
     * @param changeAmount 변경 수량
     * @param actionType "INCREASE", "DECREASE", "MOVE"
     * @param updatedBy 변경자
     */
    public void saveHistory(Inventory inventory, int changeAmount, String actionType, String updatedBy) {
        InventoryHistory history = InventoryHistory.builder()
                .inventory(inventory)
                .changeAmount(changeAmount)
                .resultQuantity(inventory.getQuantity()) // 매대 수량
                .warehouseTotal(inventory.getWarehouseQuantity()) // 창고 수량
                .minThreshold(inventory.getMinThreshold())
                .actionType(actionType)
                .remark(determineRemark(actionType))
                .updatedBy(updatedBy)
                .build();

        inventoryHistoryRepository.save(history);
    }

    /**
     * 특정 재고 ID의 이력 전체 조회
     */
    public List<InventoryHistory> findByInventoryId(Long inventoryId) {
        return inventoryHistoryRepository.findByInventory_IdOrderByCreatedAtDesc(inventoryId);
    }

    /**
     * DTO 변환 조회
     */
    public List<InventoryHistoryDto> findHistoryDtoByInventoryId(Long inventoryId) {
        return inventoryHistoryRepository.findByInventory_IdOrderByCreatedAtDesc(inventoryId)
                .stream()
                .map(history -> InventoryHistoryDto.builder()
                        .id(history.getId())
                        .itemName(history.getInventory().getItemName())
                        .changeAmount(history.getChangeAmount())
                        .resultQuantity(history.getResultQuantity())
                        .warehouseTotal(history.getWarehouseTotal())
                        .minThreshold(history.getMinThreshold())
                        .actionType(history.getActionType())
                        .remark(history.getRemark())
                        .updatedBy(history.getUpdatedBy())
                        .createdAt(history.getCreatedAt())
                        .build()
                ).toList();
    }

    /**
     * 액션 타입에 따라 비고를 자동 결정
     */
    private String determineRemark(String actionType) {
        return switch (actionType) {
            case "MOVE" -> "충전";
            case "INCREASE" -> "입고";
            case "DECREASE" -> "판매";
            default -> "";
        };
    }
}
