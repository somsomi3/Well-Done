package com.be.config.security;

import static org.springframework.security.config.Customizer.*;

import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.security.config.annotation.web.builders.HttpSecurity;
import org.springframework.security.config.annotation.web.configuration.EnableWebSecurity;
import org.springframework.security.web.SecurityFilterChain;

@Configuration
@EnableWebSecurity
public class SecurityConfig {

	@Bean
	public SecurityFilterChain securityFilterChain(HttpSecurity http) throws Exception {
		http
			.csrf(csrf -> csrf.disable()) // CSRF 비활성화
			.formLogin(withDefaults()) // 기본 로그인 폼 사용
			.authorizeHttpRequests((requests) -> requests
				.requestMatchers("/", "/mqtt").permitAll() // 루트 경로와 MQTT 엔드포인트는 인증 없이 접근 가능
				.anyRequest().authenticated()); // 다른 요청은 인증 필요
		return http.build();
	}
}
