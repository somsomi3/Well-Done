package com.be.db.repository;

import com.be.db.entity.Inventory;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.Optional;

@Repository
public interface InventoryRepository extends JpaRepository<Inventory, Long> {

    // itemName으로 재고를 찾고 싶을 때 사용
    List<Inventory> findByItemName(String itemName);

    // 재고가 임계치 이하인 것만 찾고 싶을 때
    List<Inventory> findByQuantityLessThanEqual(int threshold);
}
