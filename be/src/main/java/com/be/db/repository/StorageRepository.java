package com.be.db.repository;

import com.be.db.entity.Storage;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface StorageRepository extends JpaRepository<Storage, Long> {
    Optional<Storage> findByItemName(String itemName);

    Optional<Storage> findByItemNameAndPositionXAndPositionY(String itemName, double x, double y);

}