package com.be.domain.inventory.dto;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@AllArgsConstructor
@NoArgsConstructor
public class RobotCommandDto {
    private String command; // 예: "MOVE"
    private String target;  // 예: itemName 또는 위치
}
