package com.be.domain.storage.service;

import com.be.db.entity.Storage;
import com.be.db.repository.StorageRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import java.util.List;
import java.util.Map;
import java.util.Optional;

@Service
@RequiredArgsConstructor
public class StorageService {

    private final StorageRepository storageRepository;

    public void autoReplenishFromStorage(Long inventoryId) {
        // 예제 로직 (Inventory와 Storage의 연관 관계 가정)
        Storage storage = storageRepository.findById(inventoryId)
                .orElseThrow(() -> new RuntimeException("창고에서 상품을 찾을 수 없습니다."));

        // 좌표 정보 가정
        var from = new java.util.HashMap<String, Object>();
        from.put("x", storage.getPosition().getX());
        from.put("y", storage.getPosition().getY());

        var to = new java.util.HashMap<String, Object>();
        to.put("x", -49.30); // 매대 좌표 예시
        to.put("y", -63.09); // 매대 좌표 예시

        var command = new java.util.HashMap<String, Object>();
        command.put("command", "pick_place");
        command.put("from", from);
        command.put("to", to);
        command.put("product_id", inventoryId.toString());
        command.put("display_spot", 0); // 보충은 display_spot 무의미

        var restTemplate = new RestTemplate();
        restTemplate.postForEntity("http://localhost:8080/api/robot/pick-place", command, Map.class);
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

