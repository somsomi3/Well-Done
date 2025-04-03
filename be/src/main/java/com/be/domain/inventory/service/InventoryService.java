package com.be.domain.inventory.service;

import com.be.db.entity.Inventory;
import com.be.db.repository.InventoryRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
@RequiredArgsConstructor
public class InventoryService {

    private final InventoryRepository inventoryRepository;

    public List<Inventory> findAll() {
        return inventoryRepository.findAll();
    }

    public Inventory findById(Long id) {
        return inventoryRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("재고를 찾을 수 없습니다."));
    }

    public Inventory updateStock(Long id, int amount) {
        Inventory inventory = findById(id);
        inventory.setQuantity(inventory.getQuantity() + amount);
        return inventoryRepository.save(inventory);
    }

    public Inventory save(Inventory inventory) {
        return inventoryRepository.save(inventory); // JPA가 created_at 자동 처리
    }
    // 시뮬레이터에 로봇 이동 명령 전달
//        RobotCommandDto command = new RobotCommandDto("MOVE", inventory.getItemName());
//        simulatorSocketHandler.sendToSimulator(command);
}

