package com.be.db.entity;

import jakarta.persistence.Embeddable;
import lombok.*;

@Embeddable
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class Coordinate {
    private double x;
    private double y;
    // 로봇이 해당 좌표에서 바라볼 방향 (라디안 or 도)
    private double angle;
}
