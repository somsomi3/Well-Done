package com.be.domain.inventory.service;

import com.be.db.entity.Inventory;
import com.be.db.entity.InventoryHistory;
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

    public Inventory findById(Long id) {
        return inventoryRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("재고를 찾을 수 없습니다."));
    }

    public Inventory updateStock(Long id, int amount, String updatedBy) {
        Inventory inventory = findById(id);
        inventory.setQuantity(inventory.getQuantity() + amount);
        Inventory updated = inventoryRepository.save(inventory);
        // ⬇️ 재고 변경 이력 저장
        inventoryHistoryService.saveHistory(updated, amount, (amount > 0 ? "INCREASE" : "DECREASE"), updatedBy);

        return updated;
    }

    public Inventory save(Inventory inventory) {
        return inventoryRepository.save(inventory); // JPA가 created_at 자동 처리
    }
    // 시뮬레이터에 로봇 이동 명령 전달
//        RobotCommandDto command = new RobotCommandDto("MOVE", inventory.getItemName());
//        simulatorSocketHandler.sendToSimulator(command);
}

