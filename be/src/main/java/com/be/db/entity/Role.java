package com.be.db.entity;

import jakarta.persistence.*;
import lombok.*;

@Entity
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class Role extends BaseEntity {

	@Enumerated(EnumType.STRING)
	@Column(nullable = false, unique = true)
	private RoleName name;

	public enum RoleName {
		ROLE_USER,
		ROLE_ADMIN
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (!(o instanceof Role)) return false;
		Role role = (Role) o;
		return name == role.name;
	}

	@Override
	public int hashCode() {
		return name.hashCode();
	}
}

