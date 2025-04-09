package com.be.db.entity;

import jakarta.persistence.*;
import lombok.*;

@Entity
@Table(name = "storage")
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class Storage extends BaseEntity {

    @Column(nullable = false)
    private String itemName;

    @Column(nullable = false)
    private int quantity;

    // ⬇️ 좌표 + 각도 포함
    @Embedded
    @AttributeOverrides({
            @AttributeOverride(name = "x", column = @Column(name = "position_x")),
            @AttributeOverride(name = "y", column = @Column(name = "position_y")),
            @AttributeOverride(name = "angle", column = @Column(name = "position_angle"))
    })
    private Coordinate position;

    // 선택: 왼쪽/오른쪽 창고 줄 정보, 예: MONSHELL, COOKDAS
    @Column(name = "storage_type")
    private String storageType;
}
