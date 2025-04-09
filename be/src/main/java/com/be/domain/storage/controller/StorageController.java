package com.be.domain.storage.controller;

import com.be.db.entity.Storage;
import com.be.domain.storage.service.StorageService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/storage")
public class StorageController {

    private final StorageService storageService;

    @GetMapping("")
    public ResponseEntity<List<Storage>> getAll() {
        return ResponseEntity.ok(storageService.findAll());
    }

    @GetMapping("/{id}")
    public ResponseEntity<Storage> getOne(@PathVariable Long id) {
        return ResponseEntity.ok(storageService.findById(id));
    }

    @PostMapping
    public ResponseEntity<Storage> create(@RequestBody Storage storage) {
        return ResponseEntity.ok(storageService.save(storage));
    }

    @PutMapping("/{id}")
    public ResponseEntity<Storage> update(@PathVariable Long id, @RequestBody Storage updated) {
        Storage storage = storageService.findById(id);
        storage.setItemName(updated.getItemName());
        storage.setQuantity(updated.getQuantity());
        storage.setPosition(updated.getPosition());
        return ResponseEntity.ok(storageService.save(storage));
    }

    @DeleteMapping("/{id}")
    public ResponseEntity<Void> delete(@PathVariable Long id) {
        storageService.delete(id);
        return ResponseEntity.noContent().build();
    }
}
