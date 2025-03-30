package com.be.config;

import java.util.List;
import java.util.UUID;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.CommandLineRunner;
import org.springframework.security.crypto.bcrypt.BCryptPasswordEncoder;
import org.springframework.stereotype.Component;

import com.be.db.entity.Role;
import com.be.db.entity.User;
import com.be.db.repository.RoleRepository;
import com.be.db.repository.UserRepository;

import lombok.RequiredArgsConstructor;

@Component
@RequiredArgsConstructor
public class AdminInitializer implements CommandLineRunner {

	private final UserRepository userRepository;
	private final RoleRepository roleRepository;
	private final BCryptPasswordEncoder passwordEncoder;

	@Value("${spring.admin.username}")
	private String adminUsername;

	@Value("${spring.admin.email}")
	private String adminEmail;

	@Value("${spring.admin.password}")
	private String adminPassword;

	@Override
	public void run(String... args) {
		if (userRepository.existsByUsername(adminUsername)) return;

		Role adminRole = roleRepository.findByName(Role.RoleName.ROLE_ADMIN)
			.orElseGet(() -> roleRepository.save(Role.builder()
				.name(Role.RoleName.ROLE_ADMIN)
				.build()));

		User admin = User.builder()
			.userId(UUID.randomUUID().toString())
			.username(adminUsername)
			.email(adminEmail)
			.password(passwordEncoder.encode(adminPassword))
			.companyId("00000000")
			.roles(List.of(adminRole))
			.build();

		userRepository.save(admin);
	}
}

