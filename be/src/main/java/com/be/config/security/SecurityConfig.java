package com.be.config.security;

import java.util.List;
import java.util.Arrays;

import com.be.config.security.*;
import com.be.db.repository.UserRepository;
import com.be.domain.auth.utils.TokenUtils;
import jakarta.servlet.http.HttpServletResponse;
import lombok.extern.slf4j.Slf4j;
import org.springframework.boot.autoconfigure.security.servlet.PathRequest;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.http.HttpMethod;
import org.springframework.security.authentication.AuthenticationManager;
import org.springframework.security.config.annotation.authentication.builders.AuthenticationManagerBuilder;
import org.springframework.security.config.annotation.method.configuration.EnableMethodSecurity;
import org.springframework.security.config.annotation.web.builders.HttpSecurity;
import org.springframework.security.config.annotation.web.configuration.EnableWebSecurity;
import org.springframework.security.config.annotation.web.configuration.WebSecurityCustomizer;
import org.springframework.security.config.annotation.web.configurers.AbstractHttpConfigurer;
import org.springframework.security.config.http.SessionCreationPolicy;
import org.springframework.security.crypto.bcrypt.BCryptPasswordEncoder;
import org.springframework.security.web.SecurityFilterChain;
import org.springframework.security.web.authentication.AuthenticationFailureHandler;
import org.springframework.security.web.authentication.AuthenticationSuccessHandler;
import org.springframework.security.web.authentication.UsernamePasswordAuthenticationFilter;
import org.springframework.security.web.authentication.logout.LogoutHandler;
import org.springframework.security.web.authentication.www.BasicAuthenticationFilter;
import org.springframework.web.cors.CorsConfiguration;
import org.springframework.web.cors.CorsConfigurationSource;
import org.springframework.web.cors.UrlBasedCorsConfigurationSource;

@Slf4j
@Configuration
@EnableWebSecurity
@EnableMethodSecurity(prePostEnabled = true) // @PreAuthorize 사용을 위한 설정 추가
public class SecurityConfig {
	private final TokenUtils tokenUtils;
	private final UserRepository userRepository;

	public SecurityConfig(TokenUtils tokenUtils, UserRepository userRepository) {
		this.tokenUtils = tokenUtils;
		this.userRepository = userRepository;
	}

	@Bean
	public WebSecurityCustomizer webSecurityCustomizer() {
		return web -> web.ignoring().requestMatchers(PathRequest.toStaticResources().atCommonLocations());
	}

	@Bean
	public BCryptPasswordEncoder passwordEncoder() {
		return new BCryptPasswordEncoder();
	}

	@Bean
	public AuthenticationManager authenticationManager(HttpSecurity http) throws Exception {
		return http.getSharedObject(AuthenticationManagerBuilder.class).build();
	}

	@Bean
	public JwtAuthorizationFilter jwtAuthorizationFilter(TokenUtils tokenUtils) {
		return new JwtAuthorizationFilter(tokenUtils, userRepository);
	}

	@Bean
	public AuthenticationSuccessHandler customLoginSuccessHandler() {
		return new CustomAuthSuccessHandler(tokenUtils); // 커스텀 성공 핸들러
	}

	@Bean
	public AuthenticationFailureHandler customLoginFailureHandler() {
		return new CustomAuthFailureHandler(); // 커스텀 실패 핸들러
	}

	@Bean
	public CustomAuthenticationFilter customAuthenticationFilter(AuthenticationManager authenticationManager) {
		CustomAuthenticationFilter customAuthenticationFilter = new CustomAuthenticationFilter(authenticationManager);
		customAuthenticationFilter.setFilterProcessesUrl("/api/auth/login");

		// 핸들러 등록
		customAuthenticationFilter.setAuthenticationSuccessHandler(customLoginSuccessHandler());
		customAuthenticationFilter.setAuthenticationFailureHandler(customLoginFailureHandler());

		return customAuthenticationFilter;
	}

	@Bean
	public SecurityFilterChain securityFilterChain(
			HttpSecurity http,
			CustomAuthenticationFilter customAuthenticationFilter,
			JwtAuthorizationFilter jwtAuthorizationFilter,
			LogoutHandler customLogoutHandler,
			CustomAccessDeniedHandler customAccessDeniedHandler,
			CustomAuthenticationEntryPoint customAuthenticationEntryPoint
	) throws Exception {

		http
				.csrf(AbstractHttpConfigurer::disable)
				.cors(cors -> cors.configurationSource(corsConfigurationSource()))
				.sessionManagement(session -> session.sessionCreationPolicy(SessionCreationPolicy.STATELESS))
				.authorizeHttpRequests(auth -> auth
						.requestMatchers(
								"/api/swagger-ui/**",
								"/api/v3/api-docs/**",
								"/api/swagger-resources/**",
								"/api/webjars/**",
								"/api/auth/**",
								"/api/auth/login",
								"/api/auth/check-username",
								"/mqtt", // MQTT 엔드포인트는 인증 없이 접근 가능
								"/ws/**", // WebSocket 경로 허용 추가
								"/api/ws/**",
								"/api/robot/**",
								"/api/robot/pick-place"

						).permitAll()
						.requestMatchers(HttpMethod.OPTIONS, "/**").permitAll() // OPTIONS 메소드 허용
						.requestMatchers("/admin/**").hasRole("ADMIN") // /admin/** 경로에 접근하려면 관리자 권한이 필요
						// 공지사항 관련 권한 설정
						.requestMatchers(HttpMethod.GET, "/api/boards/announcements/**").authenticated() // 모든 사용자가 조회 가능
						.requestMatchers(HttpMethod.POST, "/api/boards/announcements/**").hasRole("ADMIN") // 관리자만 생성 가능
						.requestMatchers(HttpMethod.PUT, "/api/boards/announcements/**").hasRole("ADMIN") // 관리자만 수정 가능
						.requestMatchers(HttpMethod.DELETE, "/api/boards/announcements/**").hasRole("ADMIN") // 관리자만 삭제 가능
						.requestMatchers(HttpMethod.PATCH, "/api/boards/announcements/**").hasRole("ADMIN") // 관리자만 PATCH 가능
						.requestMatchers(HttpMethod.PATCH, "/api/boards/announcements/*/expire").hasRole("ADMIN") // 관리자만 만료일 설정 가능
						.requestMatchers(HttpMethod.POST, "/api/boards/announcements/*/view").permitAll() // 모든 사용자가 조회수 증가 가능 (인증 없이)

						.requestMatchers("/rooms/create").authenticated()
						.anyRequest().authenticated() // 다른 모든 요청은 인증 필요
				)
				.exceptionHandling(exception -> exception
						.accessDeniedHandler(customAccessDeniedHandler)
						.authenticationEntryPoint(customAuthenticationEntryPoint)
				)
				.addFilterAt(customAuthenticationFilter, UsernamePasswordAuthenticationFilter.class)
				.addFilterBefore(jwtAuthorizationFilter, BasicAuthenticationFilter.class)
				.logout(logout -> logout
						.logoutUrl("/api/auth/logout")
						.addLogoutHandler(customLogoutHandler)
						.logoutSuccessHandler((request, response, authentication) ->
								response.setStatus(HttpServletResponse.SC_OK)
						)
				);

		return http.build();
	}

	// cors 설정은 그대로 유지
	@Bean
	public CorsConfigurationSource corsConfigurationSource() {
		CorsConfiguration configuration = new CorsConfiguration();
		configuration.setAllowedOrigins(List.of("http://localhost:5173", "http://localhost:3000", "https://j12e102.p.ssafy.io"));
		configuration.setAllowedMethods(List.of("GET", "POST", "PUT", "DELETE", "PATCH", "OPTIONS"));
		configuration.setAllowedHeaders(List.of("Authorization", "x-refresh-token", "Content-Type"));
		configuration.setAllowCredentials(true); // 쿠키 허용
		configuration.setMaxAge(3600L);

		UrlBasedCorsConfigurationSource source = new UrlBasedCorsConfigurationSource();
		source.registerCorsConfiguration("/**", configuration);
		return source;
	}
}
