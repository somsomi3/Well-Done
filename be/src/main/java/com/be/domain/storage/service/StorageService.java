package com.be.domain.storage.service;

import com.be.db.entity.Inventory;
import com.be.db.entity.Storage;
import com.be.db.repository.InventoryRepository;
import com.be.db.repository.StorageRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import java.util.List;
import java.util.Map;
import java.util.Optional;

@RequiredArgsConstructor
@Service
public class StorageService {

    private final StorageRepository storageRepository;
    private final InventoryRepository inventoryRepository; // 추가

    public void autoReplenishFromStorage(String itemName) {
        List<Inventory> result = inventoryRepository.findByItemName(itemName);
        Inventory inventory = result.stream()
                .findFirst()
                .orElseThrow(() -> new RuntimeException("해당 창고 아이템이 없습니다."));

        // 👉 하드코딩된 창고 출발 좌표 (예: 쿠크다스 창고 위치)
        Map<String, Object> from = Map.of(
                "x", -60.19,
                "y", -64.82,
                "theta", Math.toRadians(-90)
        );

        // 👉 하드코딩된 도착 좌표 (예: 매대의 빈 칸 좌표)
        Map<String, Object> to = Map.of(
                "x", -50.45,
                "y", -62.56,
                "theta", Math.toRadians(0)
        );

        Map<String, Object> command = Map.of(
                "command", "pick_place",
                "from", from,
                "to", to,
                "product_id", inventory.getItemName(), // 또는 inventoryId.toString()
                "from_id", "STORAGE_COOKDAS1", // (선택사항) UI에 보여주기 위한 식별자
                "to_id", "A1", // (선택사항)
                "display_spot", 0
        );

        new RestTemplate().postForEntity("http://localhost:8080/api/robot/pick-place", command, Map.class);
    }



    public List<Storage> findAll() {
        return storageRepository.findAll();
    }

    public Storage findById(Long id) {
        return storageRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("창고 재고를 찾을 수 없습니다."));
    }

    public Storage save(Storage storage) {
        return storageRepository.save(storage);
    }

    public void delete(Long id) {
        storageRepository.deleteById(id);
    }

    public Optional<Storage> findByItemName(String itemName) {
        return storageRepository.findByItemName(itemName);
    }

    // 재고 차감
    public void decreaseQuantity(String itemName, int amount) {
        Storage storage = findByItemName(itemName)
                .orElseThrow(() -> new RuntimeException("해당 아이템을 창고에서 찾을 수 없습니다."));
        storage.setQuantity(storage.getQuantity() - amount);
        storageRepository.save(storage);
    }
}

