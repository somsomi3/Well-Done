package com.be.domain.inventory.dto;

import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

import java.time.LocalDateTime;
@Getter
@Setter
@Builder
public class InventoryHistoryDto {
    private Long id;
    private String itemName;
    private int changeAmount;
    private int resultQuantity;
    private int warehouseTotal;
    private int minThreshold;
    private String actionType;
    private String remark;
    private String updatedBy;
    private LocalDateTime createdAt;
}