package com.be.db.entity;

import com.fasterxml.jackson.annotation.JsonIgnore;
import jakarta.persistence.*;
import lombok.*;

import java.util.ArrayList;
import java.util.List;

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

    @Column(nullable = false)
    private int warehouseQuantity; // 창고 수량

    private String shelfCode;


    // 좌표 정보 (x, y, angle)
    @Embedded
    @AttributeOverrides({
            @AttributeOverride(name = "x", column = @Column(name = "x")),
            @AttributeOverride(name = "y", column = @Column(name = "y")),
            @AttributeOverride(name = "angle", column = @Column(name = "angle"))
    })
    private Coordinate coordinate;

    @OneToMany(mappedBy = "inventory", cascade = CascadeType.ALL, orphanRemoval = true)
    @JsonIgnore
    private List<InventoryHistory> historyList = new ArrayList<>();
}
