package com.be.domain.inventory.controller;

import com.be.db.entity.Inventory;
import com.be.domain.inventory.dto.InventoryDto;
import com.be.domain.inventory.service.InventoryService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.annotation.AuthenticationPrincipal;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/inventory")
public class InventoryController {

    private final InventoryService inventoryService;

    // 전체 재고 목록 조회
    @GetMapping("")
    public ResponseEntity<List<InventoryDto>> getAllInventory() {
        return ResponseEntity.ok(inventoryService.findAllDto());
    }

    // 단건 조회
    @GetMapping("/{id}")
    public ResponseEntity<Inventory> getInventory(@PathVariable Long id) {
        Inventory inventory = inventoryService.findById(id);
        return ResponseEntity.ok(inventory);
    }

    // 수량 조정

    @PostMapping("/{id}/adjust")
    public ResponseEntity<Inventory> adjustInventory(
            @PathVariable Long id,
            @RequestParam int amount,
            Authentication authentication) {

        String updatedBy = authentication.getName();

        // SecurityContextHolder에서 JWT 토큰 추출
        String token = null;
        Authentication auth = SecurityContextHolder.getContext().getAuthentication();
        if (auth != null && auth.getCredentials() instanceof String) {
            token = (String) auth.getCredentials(); // 여기서 JWT 문자열 획득
        }
        System.out.println("추출된 토큰: " + token); // null이면 문제 있음!


        Inventory updated = inventoryService.updateStock(id, amount, updatedBy, token);
        return ResponseEntity.ok(updated);
    }



    @PostMapping
    public ResponseEntity<Inventory> createInventory(@RequestBody Inventory inventory) {
        Inventory saved = inventoryService.save(inventory);
        return ResponseEntity.ok(saved);
    }

}
