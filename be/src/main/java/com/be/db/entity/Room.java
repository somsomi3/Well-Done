package com.be.db.entity;

import jakarta.persistence.*;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
// Room.java
@Entity
@Getter
@Setter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class Room extends BaseEntity {

	@Column(nullable = false, unique = true)
	private String roomId;

	@Column(name = "user_id")
	private String userId;

	@Column(name = "robot_number")
	private int robotNumber;
}
