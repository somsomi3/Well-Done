package com.be.domain.inventory.controller;

import com.be.db.entity.Inventory;
import com.be.domain.inventory.service.InventoryService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/inventory")
public class InventoryController {

    private final InventoryService inventoryService;

    // 전체 재고 목록 조회
    @GetMapping("")
    public ResponseEntity<List<Inventory>> getAllInventory() {
        List<Inventory> inventories = inventoryService.findAll();
        return ResponseEntity.ok(inventories);
    }

    // 단건 조회
    @GetMapping("/{id}")
    public ResponseEntity<Inventory> getInventory(@PathVariable Long id) {
        Inventory inventory = inventoryService.findById(id);
        return ResponseEntity.ok(inventory);
    }

    // 수량 조정
    @PostMapping("/{id}/adjust")
    public ResponseEntity<Inventory> adjustInventory(@PathVariable Long id,
                                                     @RequestParam int amount) {
        Inventory updated = inventoryService.updateStock(id, amount);
        return ResponseEntity.ok(updated);
    }

    @PostMapping
    public ResponseEntity<Inventory> createInventory(@RequestBody Inventory inventory) {
        Inventory saved = inventoryService.save(inventory);
        return ResponseEntity.ok(saved);
    }

}
