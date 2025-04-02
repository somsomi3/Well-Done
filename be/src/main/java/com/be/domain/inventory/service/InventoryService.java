package com.be.domain.inventory.service;

import com.be.db.entity.Inventory;
import com.be.db.repository.InventoryRepository;
import com.be.domain.inventory.dto.RobotCommandDto;
import com.be.domain.inventory.dto.StockAlertDto;
import com.be.domain.robot.SimulatorSocketHandler;
import lombok.RequiredArgsConstructor;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class InventoryService {

    private final InventoryRepository inventoryRepository;
    private final SimpMessagingTemplate messagingTemplate;
    private final SimulatorSocketHandler simulatorSocketHandler;

    public Inventory updateStock(Long itemId, int delta) {
        Inventory inventory = inventoryRepository.findById(itemId)
                .orElseThrow(() -> new RuntimeException("재고 없음"));

        int newQuantity = inventory.getQuantity() + delta;
        inventory.setQuantity(newQuantity);
        inventoryRepository.save(inventory);

        if (newQuantity <= inventory.getMinThreshold()) {
            notifyStockShortage(inventory);
        }

        return inventory;
    }

    private void notifyStockShortage(Inventory inventory) {
        // 프론트에 재고 부족 알림
        messagingTemplate.convertAndSend("/topic/stock-alert",
                new StockAlertDto(inventory.getItemName(), inventory.getQuantity()));

        // 시뮬레이터에 로봇 이동 명령 전달
//        RobotCommandDto command = new RobotCommandDto("MOVE", inventory.getItemName());
//        simulatorSocketHandler.sendToSimulator(command);
    }
    public Inventory findById(Long id) {
        return inventoryRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("재고를 찾을 수 없습니다."));
    }
}

