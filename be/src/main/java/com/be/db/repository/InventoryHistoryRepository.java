package com.be.db.repository;

import com.be.db.entity.InventoryHistory;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;

public interface InventoryHistoryRepository extends JpaRepository<InventoryHistory, Long> {
    List<InventoryHistory> findByInventory_IdOrderByCreatedAtDesc(Long inventoryId);
}

