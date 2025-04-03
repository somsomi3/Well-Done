package com.be.domain.inventory.dto;

import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class StockAlertDto {
    private String itemName;
    private int currentQuantity;
}
