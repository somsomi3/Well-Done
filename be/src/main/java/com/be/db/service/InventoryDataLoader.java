package com.be.db.service;

import com.be.db.entity.Coordinate;
import com.be.db.entity.Inventory;
import com.be.db.repository.InventoryRepository;
import com.be.db.repository.StorageRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.boot.CommandLineRunner;
import org.springframework.core.annotation.Order;
import org.springframework.stereotype.Component;

import java.util.ArrayList;
import java.util.List;

@Component
@RequiredArgsConstructor
@Order(2)
public class InventoryDataLoader implements CommandLineRunner {

    private final InventoryRepository inventoryRepository;
    private final StorageRepository storageRepository;

    @Override
    public void run(String... args) {
        List<Inventory> inventories = new ArrayList<>();

        // 과자명, 전시 위치, 전시 각도 (x, y, angle)
        Object[][] displaySpots = {
                {"A1", "빼빼로", -47.36, -63.70, 180},
                {"B1", "카스타드", -51.16, -63.69, 180},
                {"C1", "오뜨", -48.73, -63.07, 90},
                {"D1", "쿠크다스", -50.45, -62.56, -90},
                {"E1", "화이트하임", -47.38, -61.98, 0},
                {"F1", "뽀또", -51.17, -62.01, 0},
                {"G1", "마가렛트", -47.40, -60.27, 180},
                {"H1", "빠다코코낫", -51.20, -60.17, 180},
                {"I1", "몽쉘", -48.75, -59.40, 90},
                {"I2", "오예스", -48.73, -58.92, 90},
                {"K1", "에이스", -47.39, -58.01, 0},
                {"L1", "버터링", -51.26, -57.97, 0},

                {"EMP1", "빈파레트저장소", -55.56, -66.29, 0},
                {"EMP2", "임시저장소", -55.03, -66.50, 0},

        };

        //재고 보충 시, 로봇이 가져오는 보관 장소
        Coordinate warehouseMonshell = new Coordinate(-59.20, -64.82, -90);
        Coordinate warehouseCookdas = new Coordinate(-60.19, -64.82, -90);
        Coordinate defaultWarehouse = new Coordinate(-55.00, -65.00, 0);

        for (Object[] entry : displaySpots) {
            String shelfCode = (String) entry[0];
            String name = (String) entry[1];

            // 이미 등록된 상품이면 스킵
            if (inventoryRepository.findByItemName(name).isPresent()) {
                continue;
            }

            double x = ((Number) entry[2]).doubleValue();
            double y = ((Number) entry[3]).doubleValue();
            double angle = ((Number) entry[4]).doubleValue();

            Coordinate display = new Coordinate(x, y, angle);
            Coordinate warehouse;


            if (name.equals("몽쉘")) warehouse = warehouseMonshell;
            else if (name.equals("쿠크다스")) warehouse = warehouseCookdas;
            else if (name.equals("빈 파레트 저장소")) warehouse = new Coordinate(-55.56, -66.29, 0); // 빈 파레트 저장소
            else if (name.equals("임시 저장소")) warehouse = new Coordinate(-55.03, -66.50, 0); // 임시 저장소
            else warehouse = defaultWarehouse;
            int warehouseQuantity = storageRepository.getTotalQuantityByItemName(name);
            inventories.add(Inventory.builder()
                    .itemName(name)
                    .shelfCode(shelfCode)
                    .quantity(50)
                    .minThreshold(0)
                    .warehouseQuantity(warehouseQuantity)
                    .coordinate(display)
                    .build());
        }

        inventoryRepository.saveAll(inventories);
        System.out.println("Inventory 전시 좌표 초기화 완료");
    }
}
