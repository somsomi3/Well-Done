package com.be.db.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Table;
import lombok.*;

@Entity
@Table(name = "robot_path")
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class RobotPath extends BaseEntity {

	@Column(nullable = false)
	private Long robotId;

	@Column(nullable = false)
	private double x;

	@Column(nullable = false)
	private double y;

	// 필요 시 toString도 추가 가능
	@Override
	public String toString() {
		return "RobotPath{" +
			"robotId=" + robotId +
			", x=" + x +
			", y=" + y +
			", createdAt=" + getCreatedAt() +
			'}';
	}
}
