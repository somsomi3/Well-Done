package com.be.domain.auth.controller;

import com.be.db.entity.User;
import com.be.db.entity.Role;
import com.be.db.repository.UserRepository;
import com.be.db.repository.RoleRepository;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;

import lombok.RequiredArgsConstructor;

@RestController
@RequestMapping("/admin")
@RequiredArgsConstructor
public class AdminController {

	private final UserRepository userRepository;
	private final RoleRepository roleRepository;

	// 1. 모든 사용자 조회
	@GetMapping("/users")
	public ResponseEntity<List<User>> getAllUsers() {
		return ResponseEntity.ok(userRepository.findAll());
	}

	// 2. 특정 사용자 조회
	@GetMapping("/users/{username}")
	public ResponseEntity<User> getUserByUsername(@PathVariable String username) {
		Optional<User> user = userRepository.findByUsername(username);
		if (!user.isPresent()) {
			return ResponseEntity.status(404).body(null);
		}
		return ResponseEntity.ok(user.get());
	}

	// 3. 사용자 권한 부여 (관리자 권한 부여)
	@PutMapping("/users/{username}/grant-admin")
	public ResponseEntity<String> grantAdmin(@PathVariable String username) {
		Optional<User> user = userRepository.findByUsername(username);
		if (!user.isPresent()) {
			return ResponseEntity.status(404).body("User not found");
		}

		Role adminRole = roleRepository.findByName(Role.RoleName.ROLE_ADMIN)
			.orElseThrow(() -> new RuntimeException("Role not found"));

		User existingUser = user.get();
		existingUser.getRoles().add(adminRole);
		userRepository.save(existingUser);
		return ResponseEntity.ok("Admin role granted to user");
	}

	// 4. 사용자 삭제 (강제 탈퇴)
	@DeleteMapping("/users/{username}")
	public ResponseEntity<String> deleteUser(@PathVariable String username) {
		Optional<User> user = userRepository.findByUsername(username);
		if (!user.isPresent()) {
			return ResponseEntity.status(404).body("User not found");
		}

		userRepository.delete(user.get());
		return ResponseEntity.ok("User deleted successfully");
	}

	// 5. 관리자 권한 삭제
	@PutMapping("/users/{username}/revoke-admin")
	public ResponseEntity<String> revokeAdmin(@PathVariable String username) {
		Optional<User> user = userRepository.findByUsername(username);
		if (!user.isPresent()) {
			return ResponseEntity.status(404).body("User not found");
		}

		User existingUser = user.get();
		Role adminRole = roleRepository.findByName(Role.RoleName.ROLE_ADMIN)
			.orElseThrow(() -> new RuntimeException("Role not found"));

		existingUser.getRoles().removeIf(role -> role.getName() == Role.RoleName.ROLE_ADMIN); // 중요

		userRepository.save(existingUser);
		return ResponseEntity.ok("Admin role revoked from user");
	}

}