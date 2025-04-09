package com.be.db.service;

import com.be.db.entity.Coordinate;
import com.be.db.entity.Storage;
import com.be.db.repository.StorageRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.boot.CommandLineRunner;
import org.springframework.core.annotation.Order;
import org.springframework.stereotype.Component;

import java.util.ArrayList;
import java.util.List;

@Component
@RequiredArgsConstructor
@Order(1)
public class StorageDataLoader implements CommandLineRunner {

    private final StorageRepository storageRepository;

    @Override
    public void run(String... args) {
        List<Storage> storages = new ArrayList<>();

        // 몽쉘 좌측 창고
        double xMon = -59.20;
        double[] yMonshells = {-64.82, -64.32, -63.82, -63.32, -62.82, -62.32, -61.82, -61.32, -60.82, -60.32};

        for (int i = 0; i < yMonshells.length; i++) {
            double y = yMonshells[i];
            String shelfCode = "LST" + (i + 1);  // LST1 ~ LST10

            if (storageRepository.findByItemNameAndPositionXAndPositionY("몽쉘", xMon, y).isPresent()) continue;

            storages.add(Storage.builder()
                    .itemName("몽쉘")
                    .quantity(25)
//                    .storageType("MONSHELL")
                    .shelfCode(shelfCode)
                    .position(new Coordinate(xMon, y, -90))
                    .build());
        }

        // 쿠크다스 우측 창고
        double xCook = -60.19;
        double[] yCooks = {-64.82, -64.32, -63.82, -63.32, -62.82, -62.32, -61.82, -61.32, -60.82, -60.32};

        for (int i = 0; i < yCooks.length; i++) {
            double y = yCooks[i];
            String shelfCode = "RST" + (i + 1);  // RST1 ~ RST10

            if (storageRepository.findByItemNameAndPositionXAndPositionY("쿠크다스", xCook, y).isPresent()) continue;

            storages.add(Storage.builder()
                    .itemName("쿠크다스")
                    .quantity(25)
//                    .storageType("COOKDAS")
                    .shelfCode(shelfCode)
                    .position(new Coordinate(xCook, y, 90))
                    .build());
        }

        storageRepository.saveAll(storages);
        System.out.println("창고 위치 + 선반 코드까지 초기화 완료");
    }

}