package com.be.domain.auth.controller;

import com.be.common.model.response.BaseResponseBody;
import com.be.config.security.CustomLogoutHandler;
import com.be.domain.auth.request.LoginRequest;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.web.bind.annotation.*;

import com.be.db.entity.User;
import com.be.db.repository.UserRepository;
import com.be.domain.auth.request.RegisterRequest;
import com.be.domain.auth.response.RegisterResponse;
import com.be.domain.auth.utils.TokenUtils;

import java.util.UUID;

@Slf4j
@RestController
@RequestMapping("/auth")
@RequiredArgsConstructor
public class AuthController {

    private final UserRepository userRepository;
    private final TokenUtils tokenUtils;
    private final PasswordEncoder passwordEncoder;
    private final CustomLogoutHandler customLogoutHandler;

    // 1. 회원가입
    @PostMapping("/register")
    public ResponseEntity<BaseResponseBody<RegisterResponse>> register(@Valid @RequestBody RegisterRequest request) {
        log.info("회원가입 요청: {}", request.getEmail());

        // username 중복 검사
        if (userRepository.existsByUsername(request.getUsername())) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST)
                    .body(BaseResponseBody.error(400, "이미 사용 중인 사용자 이름입니다."));
        }

        if (userRepository.existsByEmail(request.getEmail())) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST)
                    .body(BaseResponseBody.error(400, "이미 사용 중인 이메일입니다."));
        }

        String encodedPassword = passwordEncoder.encode(request.getPassword());
        String generatedUserId = UUID.randomUUID().toString();

        User user = new User();
        user.setUserId(generatedUserId);
        user.setEmail(request.getEmail());
        user.setPassword(encodedPassword);
        user.setCompanyId(request.getCompanyId());
        user.setUsername(request.getUsername());
        userRepository.save(user);

        return ResponseEntity.ok(BaseResponseBody.success(
                new RegisterResponse(user.getId(), user.getEmail(), user.getUsername()), "회원가입 완료"));
    }

    // 2. 로그인
    @PostMapping("/test-login")
    public ResponseEntity<String> testLogin(@RequestBody LoginRequest loginRequest) {
        log.info("테스트 로그인 요청 수신: {}", loginRequest.getUsername());
        return ResponseEntity.ok("테스트용 로그인 요청 수신: " + loginRequest.getUsername());
    }
//    @PostMapping("/test-login")
//    public ResponseEntity<String> testLogin(@RequestBody LoginRequest loginRequest) {
//        return ResponseEntity.ok("테스트용 로그인 요청 수신: " + loginRequest.getUsername());
//    }

    //  3. 로그아웃 => CustomLogoutHandler에서 바로 처리
//    @PostMapping("/logout")
//    public ResponseEntity<BaseResponseBody<Void>> logout(HttpServletRequest request, HttpServletResponse response) {
//        log.info("[+] 로그아웃 요청");
//        customLogoutHandler.logout(request, response, null);
//        return ResponseEntity.ok(BaseResponseBody.success(null, "로그아웃 완료"));
//    }

    // 4. 토큰 갱신
    @PostMapping("/refresh")
    public ResponseEntity<BaseResponseBody<String>> refreshToken(
            @CookieValue(value = "refreshToken", required = false) String refreshToken) {

        log.info("토큰 갱신 요청");

        if (refreshToken == null) {
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED)
                    .body(BaseResponseBody.error(401, "리프레시 토큰이 없습니다."));
        }
        log.info("토큰 갱신 요청2");
        String newAccessToken;
        try {
            newAccessToken = tokenUtils.validateAndGenerateNewAccessToken(refreshToken);
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED)
                    .body(BaseResponseBody.error(401, "리프레시 토큰이 유효하지 않습니다."));
        }
        log.info("토큰 갱신 요청3");
        return ResponseEntity.ok(BaseResponseBody.success(newAccessToken, "토큰 재발급 완료"));
    }
}
