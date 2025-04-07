package com.be.domain.inventory.dto;

import lombok.*;

@Getter
@Setter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class InventoryDto {
    private Long id;
    private String itemName;
    private int quantity;
    private int warehouseQuantity;
    private int minThreshold;
}

