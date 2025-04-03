package com.be.db.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Table;
import lombok.*;

@Entity
@Table(name = "inventory")
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class Inventory extends BaseEntity {

    @Column(nullable = false)
    private String itemName;

    @Column(nullable = false)
    private int quantity; // 현재 재고 수량

    @Column(nullable = false)
    private int minThreshold; // 재고 부족 기준 수량

    // 필요하면 창고 위치, 카테고리 등도 추가 가능
    // @Column
    // private String location;
}
