package com.be.db.service;

import com.be.db.repository.UserRepository;
import com.be.domain.auth.dto.UserDto;
import com.be.domain.auth.request.LoginRequest;
import com.be.domain.auth.request.RegisterRequest;
import com.be.domain.auth.response.LoginResponse;
import com.be.domain.auth.response.RegisterResponse;
import com.be.db.entity.User;
import com.be.domain.auth.utils.TokenUtils;
import jakarta.servlet.http.HttpServletResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.security.crypto.bcrypt.BCryptPasswordEncoder;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;
import org.springframework.web.server.ResponseStatusException;

import java.util.Optional;

@Slf4j
@Service
@RequiredArgsConstructor
public class AuthService {

    private final UserRepository userRepository;
    private final PasswordEncoder passwordEncoder;
    private final TokenUtils tokenUtils;

    /**
     * 🔹 회원가입
     */
    public RegisterResponse register(RegisterRequest request) {
        if (userRepository.existsByEmail(request.getEmail())) {
            throw new ResponseStatusException(HttpStatus.BAD_REQUEST, "이미 사용 중인 이메일입니다.");
        }

        User user = userRepository.save(User.builder()
                .email(request.getEmail())
                .password(passwordEncoder.encode(request.getPassword()))
                .username(request.getUsername())
                .build());

        return new RegisterResponse(user.getId(), user.getEmail(), user.getUsername());
    }

    /**
     * 🔹 로그인 (JWT 발급)
     */
    public LoginResponse login(LoginRequest request) {
        // 🔸 username으로 유저 조회
        User user = userRepository.findByUsername(request.getUsername())
                .orElseThrow(() -> new ResponseStatusException(HttpStatus.UNAUTHORIZED, "아이디가 존재하지 않습니다."));

        // 🔸 비밀번호 검증
        if (!passwordEncoder.matches(request.getPassword(), user.getPassword())) {
            throw new ResponseStatusException(HttpStatus.UNAUTHORIZED, "비밀번호가 일치하지 않습니다.");
        }

        return new LoginResponse(
                tokenUtils.generateJwt(UserDto.builder().build()),
                user.getEmail(),
                user.getUsername()
        );
    }


    /**
     * 🔹 토큰 갱신
     */
    public String refreshToken(String refreshToken) {
        if (refreshToken == null || !tokenUtils.isValidToken(refreshToken).isValid()) {
            throw new ResponseStatusException(HttpStatus.UNAUTHORIZED, "리프레시 토큰이 유효하지 않습니다.");
        }

        return tokenUtils.validateAndGenerateNewAccessToken(refreshToken);
    }
}
