package com.be.db.service;

import com.be.db.entity.Coordinate;
import com.be.db.entity.Storage;
import com.be.db.repository.StorageRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.boot.CommandLineRunner;
import org.springframework.stereotype.Component;

import java.util.ArrayList;
import java.util.List;

@Component
@RequiredArgsConstructor
public class StorageDataLoader implements CommandLineRunner {

    private final StorageRepository storageRepository;

    @Override
    public void run(String... args) {
        List<Storage> storages = new ArrayList<>();

        // 몽쉘 좌표
        double xMon = -59.20;
        double[] yMonshells = {-64.82, -64.32, -63.82, -63.32, -62.82, -62.32, -61.82, -61.32, -60.82, -60.32};

        for (double y : yMonshells) {
            if (storageRepository.findByItemNameAndPositionXAndPositionY("몽쉘", xMon, y).isPresent()) continue;

            storages.add(Storage.builder()
                    .itemName("몽쉘")
                    .quantity(1)
                    .storageType("MONSHELL")
                    .position(new Coordinate(xMon, y, -90))
                    .build());
        }

        // 쿠크다스 좌표
        double xCook = -60.19;
        double[] yCooks = {-64.82, -64.32, -63.82, -63.32, -62.82, -62.32, -61.82, -61.32, -60.82, -60.32};

        for (double y : yCooks) {
            if (storageRepository.findByItemNameAndPositionXAndPositionY("쿠크다스", xCook, y).isPresent()) continue;

            storages.add(Storage.builder()
                    .itemName("쿠크다스")
                    .quantity(1)
                    .storageType("COOKDAS")
                    .position(new Coordinate(xCook, y, -90))
                    .build());
        }

        storageRepository.saveAll(storages);
        System.out.println("✅ 창고 위치 중복 없이 초기화 완료");
    }

}