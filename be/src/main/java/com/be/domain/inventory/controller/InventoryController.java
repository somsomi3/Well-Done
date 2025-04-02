package com.be.domain.inventory.controller;

import com.be.db.entity.Inventory;
import com.be.db.repository.InventoryRepository;
import com.be.domain.inventory.service.InventoryService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/inventory")
public class InventoryController {

    private final InventoryService inventoryService;
    private final InventoryRepository inventoryRepository;

    @GetMapping("/{id}")
    public ResponseEntity<Inventory> getInventory(@PathVariable Long id) {
        Inventory inventory = inventoryService.findById(id);
        return ResponseEntity.ok(inventory);
    }

    @PostMapping("/{id}/adjust")
    public ResponseEntity<Inventory> adjustInventory(@PathVariable Long id,
                                                     @RequestParam int amount) {
        Inventory updated = inventoryService.updateStock(id, amount);
        return ResponseEntity.ok(updated);
    }
    public Inventory findById(Long id) {
        return inventoryRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("재고를 찾을 수 없습니다."));
    }
}
