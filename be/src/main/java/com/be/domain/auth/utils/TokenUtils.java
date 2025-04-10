package com.be.domain.auth.utils;

import com.be.db.entity.User;
import com.be.domain.auth.dto.UserDto;
import io.jsonwebtoken.*;
import io.jsonwebtoken.security.Keys;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

import javax.crypto.SecretKey;
import java.nio.charset.StandardCharsets;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Slf4j
@Component
public class TokenUtils {

    private final SecretKey JWT_SECRET_KEY;

    // SecretKey 주입받도록 수정
    public TokenUtils(@Value("${spring.jwt.secret}") String jwtSecretKey) {
        this.JWT_SECRET_KEY = Keys.hmacShaKeyFor(jwtSecretKey.getBytes(StandardCharsets.UTF_8));
    }

    /**
     * 🔹 userId만 받아서 Access Token 생성
     */
    // public String generateAccessToken(Long userId, String username) {
    //     UserDto userDto = UserDto.builder()
    //             .userid(userId)
    //             .username(username) // 꼭 포함시켜야 Claims에도 들어감
    //         .roles(role)
    //             .build();
    //     return generateJwt(userDto);
    // }
    public String generateAccessToken(UserDto userDto) {
        return generateJwt(userDto);
    }

    /**
     * 🔹 JWT 만료 날짜 생성
     */
    private Date createExpiredDate(boolean isAccessToken) {
        Calendar c = Calendar.getInstance();
        if (isAccessToken) {
            c.add(Calendar.DATE, 7);  // AccessToken: 15분
        } else {
            c.add(Calendar.DATE, 14);    // RefreshToken: 14일
        }
        return c.getTime();
    }

    /**
     * 🔹 JWT Claims 생성
     */
    private Map<String, Object> createClaims(UserDto userDto, boolean isAccessToken) {
        Map<String, Object> claims = new HashMap<>();
        claims.put("userId", userDto.getUserid());
        claims.put("username", userDto.getUsername()); // 모든 토큰에 사용자명 포함

        // [수정] 액세스 토큰에는 사용자 역할 정보도 추가
        if (isAccessToken && userDto.getRoles() != null) {
            claims.put("roles", userDto.getRoles());
        }

        return claims;
    }

    /**
     * 🔹 JWT 토큰 생성 (AccessToken)
     */
    public String generateJwt(UserDto userDto) {
        return Jwts.builder()
            .subject(userDto.getEmail())
            .claim("userId", userDto.getUserid())
            .claim("username", userDto.getUsername())
            .claim("roles", userDto.getRoles())
            .signWith(JWT_SECRET_KEY, Jwts.SIG.HS256)
            .expiration(createExpiredDate(true))
            .compact();
    }

    /**
     * 🔹 JWT 리프레시 토큰 생성
     */
    public String generateRefreshToken(UserDto userDto) {
        return Jwts.builder()
            .claims(createClaims(userDto, false))
            .subject(userDto.getUsername())
            .signWith(JWT_SECRET_KEY, Jwts.SIG.HS256)
            .expiration(createExpiredDate(false))
            .compact();
    }

    /**
     * 🔹 JWT 토큰 유효성 검사
     */
    public ValidTokenDto isValidToken(String token) {
        try {
            Claims claims = getTokenToClaims(token);
            log.info("JWT 만료 시간: {}", claims.getExpiration());
            log.info("User ID: {}", claims.get("userId"));
            log.info("Username: {}", claims.get("username"));
            return ValidTokenDto.valid();  // builder 없이 객체 생성
        } catch (ExpiredJwtException e) {
            log.error("토큰 만료됨", e);
            return ValidTokenDto.expired();
        } catch (JwtException e) {
            log.error("토큰 위조됨", e);
            return ValidTokenDto.invalid();
        } catch (NullPointerException e) {
            log.error("토큰이 null입니다.", e);
            return ValidTokenDto.nullToken();
        }
    }

    /**
     * 🔹 Authorization 헤더에서 Bearer 토큰 추출
     */
    public static String getHeaderToToken(String header) {
        if (header != null) {
            header = header.trim(); // 앞뒤 공백 제거

            // Bearer 토큰 분리 (대소문자 무시)
            String[] parts = header.split("\\s+"); // 하나 이상의 공백
            if (parts.length == 2 && parts[0].equalsIgnoreCase("Bearer")) {
                return parts[1];
            }
        }
        throw new IllegalArgumentException("잘못된 Authorization 헤더 형식 현재 값: [" + header + "]");
    }

    /**
     * 🔹 JWT 토큰을 Claims 객체로 변환
     */
    public Claims getTokenToClaims(String token) {
        return Jwts.parser()
            .verifyWith(JWT_SECRET_KEY)
            .build()
            .parseSignedClaims(token)
            .getPayload();
    }

    /**
     * 🔹 JWT Claims에서 userId 가져오기
     */
    public String getClaimsToUserId(String token) {
        Claims claims = getTokenToClaims(token);
        return claims.get("userId").toString();
    }

    /**
     * 🔹 JWT Claims에서 UserDto 생성
     */
    public UserDto getClaimsToUserDto(String token, boolean isAccessToken) {
        Claims claims = getTokenToClaims(token);
        Long userId = Long.valueOf(claims.get("userId").toString());

        // [수정] 모든 토큰에서 username 추출
        String username = claims.getOrDefault("username", "").toString();

        // [수정] 빌더 시작
        UserDto.UserDtoBuilder builder = UserDto.builder()
            .userid(userId)
            .username(username);

        // [수정] roles 클레임이 있으면 추가
        if (claims.get("roles") != null) {
            builder.roles((List<String>)claims.get("roles"));
        }

        return builder.build();
    }

    /**
     * 🔹 리프레시 토큰 검증 후 새로운 액세스 토큰 발급
     */
    public String validateAndGenerateNewAccessToken(String refreshToken) {
        Claims claims = getTokenToClaims(refreshToken);

        Long userId = Long.parseLong(claims.get("userId").toString());
        Object usernameClaim = claims.get("username");

        if (usernameClaim == null) {
            throw new IllegalArgumentException("RefreshToken에서 username이 존재하지 않습니다!");
        }

        String username = usernameClaim.toString();

        // [수정] 역할 정보 추가
        UserDto.UserDtoBuilder builder = UserDto.builder()
            .userid(userId)
            .username(username);

        // [수정] 리프레시 토큰에 roles 클레임이 있으면 추가
        if (claims.get("roles") != null) {
            builder.roles((List<String>)claims.get("roles"));
        }

        return generateJwt(builder.build());
    }
}
