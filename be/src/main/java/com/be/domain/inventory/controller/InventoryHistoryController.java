package com.be.domain.inventory.controller;

import com.be.db.entity.InventoryHistory;
import com.be.domain.inventory.dto.InventoryHistoryDto;
import com.be.domain.inventory.service.InventoryHistoryService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/inventory")
public class InventoryHistoryController {

    private final InventoryHistoryService inventoryHistoryService;

    @GetMapping("/{inventoryId}/history")
    public ResponseEntity<List<InventoryHistoryDto>> getInventoryHistory(@PathVariable Long inventoryId) {
        List<InventoryHistoryDto> historyList = inventoryHistoryService.findHistoryDtoByInventoryId(inventoryId);
        return ResponseEntity.ok(historyList);
    }

}
