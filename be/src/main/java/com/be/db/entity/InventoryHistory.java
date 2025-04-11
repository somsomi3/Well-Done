package com.be.db.entity;

import jakarta.persistence.*;
import lombok.*;

@Entity
@Table(name = "inventory_history")
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class InventoryHistory extends BaseEntity {

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "inventory_id")
    private Inventory inventory;

    @Column(nullable = false)
    private String itemName; // 상품명 (추가)

    @Column(nullable = false)
    private int changeAmount;

    @Column(nullable = false)
    private int resultQuantity;

    @Column(nullable = false)
    private int warehouseTotal; // 창고 총량 (추가)

    @Column(nullable = false)
    private int minThreshold; // 매대 최소 수량 (추가)

    @Column(nullable = false)
    private String actionType; // "INCREASE", "DECREASE", "MOVE"

    @Column(length = 255)
    private String remark; // 비고 (예: 판매, 입고, 충전 등)

    @Column(nullable = false)
    private String updatedBy;
}
