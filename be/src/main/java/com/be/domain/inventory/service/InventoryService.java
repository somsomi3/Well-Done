package com.be.domain.inventory.service;

import com.be.db.entity.Inventory;
import com.be.db.repository.InventoryRepository;
import com.be.domain.inventory.dto.InventoryDto;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
@RequiredArgsConstructor
public class InventoryService {

    private final InventoryRepository inventoryRepository;
    private final InventoryHistoryService inventoryHistoryService;

    // 모든 재고를 DTO로 반환
    public List<InventoryDto> findAllDto() {
        return inventoryRepository.findAll().stream()
                .map(i -> InventoryDto.builder()
                        .id(i.getId())
                        .itemName(i.getItemName())
                        .quantity(i.getQuantity())
                        .warehouseQuantity(i.getWarehouseQuantity())
                        .minThreshold(i.getMinThreshold())
                        .build())
                .toList();
    }

    // 단건 조회
    public Inventory findById(Long id) {
        return inventoryRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("재고를 찾을 수 없습니다."));
    }

    // 수량 조정 및 이력 저장 (로봇 명령은 프론트에서 따로 요청)
    public Inventory updateStock(Long id, int amount, String updatedBy, String token) {
        Inventory inventory = findById(id);
        inventory.setQuantity(inventory.getQuantity() + amount);
        Inventory updated = inventoryRepository.save(inventory);

        // 재고 이력 저장
        inventoryHistoryService.saveHistory(
                updated,
                amount,
                (amount > 0 ? "INCREASE" : "DECREASE"),
                updatedBy
        );

        return updated;
    }

    // 신규 재고 등록
    public Inventory save(Inventory inventory) {
        return inventoryRepository.save(inventory);
    }
}
