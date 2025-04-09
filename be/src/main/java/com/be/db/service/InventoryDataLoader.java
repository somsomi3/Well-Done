package com.be.db.service;

import com.be.db.entity.Coordinate;
import com.be.db.entity.Inventory;
import com.be.db.repository.InventoryRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.boot.CommandLineRunner;
import org.springframework.stereotype.Component;

import java.util.ArrayList;
import java.util.List;

@Component
@RequiredArgsConstructor
public class InventoryDataLoader implements CommandLineRunner {

    private final InventoryRepository inventoryRepository;

    @Override
    public void run(String... args) {
        List<Inventory> inventories = new ArrayList<>();

        // 과자명, 전시 위치, 전시 각도 (x, y, angle)
        Object[][] displaySpots = {
                {"빼빼로", -47.36, -63.70, 180},
                {"카스타드", -51.16, -63.69, 180},
                {"오뜨", -48.73, -63.07, 90},
                {"쿠크다스", -50.45, -62.56, -90},
                {"화이트하임", -47.38, -61.98, 0},
                {"뽀또", -51.17, -62.01, 0},
                {"마가렛트", -47.40, -60.27, 180},
                {"빠다코코낫", -51.20, -60.17, 180},
                {"몽쉘", -48.75, -59.40, 90},
                {"오예스", -48.73, -58.92, 90},
                {"에이스", -47.39, -58.01, 0},
                {"버터링", -51.26, -57.97, 0},
        };

        Coordinate warehouseMonshell = new Coordinate(-59.20, -64.82, -90);
        Coordinate warehouseCookdas = new Coordinate(-60.19, -64.82, -90);
        Coordinate defaultWarehouse = new Coordinate(-55.00, -65.00, 0);

        for (Object[] entry : displaySpots) {
            String name = (String) entry[0];


            // 이미 등록된 상품이면 스킵
            if (inventoryRepository.findByItemName(name).isPresent()) {
                continue;
            }

            double x = ((Number) entry[1]).doubleValue();
            double y = ((Number) entry[2]).doubleValue();
            double angle = ((Number) entry[3]).doubleValue();

            Coordinate display = new Coordinate(x, y, angle);
            Coordinate warehouse;

            if (name.equals("몽쉘")) warehouse = warehouseMonshell;
            else if (name.equals("쿠크다스")) warehouse = warehouseCookdas;
            else warehouse = defaultWarehouse;

            inventories.add(Inventory.builder()
                    .itemName(name)
                    .quantity(0)
                    .minThreshold(5)
                    .warehouseQuantity(1)
                    .coordinate(display)
                    .build());
        }

        inventoryRepository.saveAll(inventories);
        System.out.println("Inventory 전시 좌표 초기화 완료");
    }
}
