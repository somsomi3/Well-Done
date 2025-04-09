package com.be.db.repository;

import com.be.db.entity.Storage;
import io.lettuce.core.dynamic.annotation.Param;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.Optional;

public interface StorageRepository extends JpaRepository<Storage, Long> {
    Optional<Storage> findByItemName(String itemName);

    Optional<Storage> findByItemNameAndPositionXAndPositionY(String itemName, double x, double y);


    @Query("SELECT COALESCE(SUM(s.quantity), 0) FROM Storage s WHERE s.itemName = :itemName")
    int getTotalQuantityByItemName(@Param("itemName") String itemName);
}