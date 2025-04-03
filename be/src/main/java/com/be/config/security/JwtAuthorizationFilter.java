package com.be.config.security;

import com.be.db.entity.User;
import com.be.db.repository.UserRepository;
import com.be.domain.auth.dto.UserDto;
import com.be.domain.auth.service.TokenBlackListService;
import com.be.domain.auth.utils.TokenUtils;
import com.be.domain.auth.utils.ValidTokenDto;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import io.jsonwebtoken.ExpiredJwtException;
import io.jsonwebtoken.JwtException;

import io.micrometer.common.util.StringUtils;
import jakarta.servlet.FilterChain;
import jakarta.servlet.ServletException;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import lombok.NonNull;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.security.authentication.UsernamePasswordAuthenticationToken;
import org.springframework.security.core.authority.SimpleGrantedAuthority;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.stereotype.Component;
import org.springframework.web.filter.OncePerRequestFilter;

//토큰 검증
@Slf4j
@Component
public class JwtAuthorizationFilter extends OncePerRequestFilter {

    @Autowired
    private TokenBlackListService tokenBlackListService;
    private final TokenUtils tokenUtils;

    private final UserRepository userRepository;

    private static final String HTTP_METHOD_OPTIONS = "OPTIONS";
    private static final String ACCESS_TOKEN_HEADER_KEY = "Authorization";
    private static final String REFRESH_TOKEN_HEADER_KEY = "x-refresh-token";
    private static final List<String> WHITELIST_PREFIXES = List.of(
            "/api/swagger-ui/index.html",
            "/api/auth/login",
            "/api/auth/register",
            "/api/auth/refresh",
            "/api/v3/api-docs",
            "/api/swagger-ui",
            "/api/swagger-ui/**",
            "/api/swagger-resources/**",
            "/api/webjars/**",
            "/api/auth/test-login",
            "/api/auth/check-username",
            "/api/auth/logout",
            "/ws/**",
            "/api/ws/**"
    );

    public JwtAuthorizationFilter(TokenUtils tokenUtils, UserRepository userRepository) {
        this.tokenUtils = tokenUtils;
        this.userRepository = userRepository;
    }

    //클라이언트가 보낸 요청 헤더에서 Access Token을 꺼내서 검증하고,
    //검증에 성공하면 SecurityContext에 인증 정보를 저장
    @Override
    protected void doFilterInternal(HttpServletRequest request,
                                    @NonNull HttpServletResponse response,
                                    @NonNull FilterChain chain)
            throws IOException, ServletException {

        String requestURI = request.getRequestURI();

        // WebSocket이나 화이트리스트에 해당하는 경로는 검증 없이 통과
        if (requestURI.startsWith("/ws/") || isWhitelisted(requestURI) ||
            HTTP_METHOD_OPTIONS.equalsIgnoreCase(request.getMethod())) {
            log.debug("JWT 필터 제외 경로: {}", requestURI);
            chain.doFilter(request, response);
            return; // 여기서 확실히 리턴
        }

        try {
            String accessTokenHeader = request.getHeader(ACCESS_TOKEN_HEADER_KEY);
            String refreshTokenHeader = request.getHeader(REFRESH_TOKEN_HEADER_KEY);

            System.out.println("Authorization Header: " + accessTokenHeader);
            System.out.println("Refresh Header: " + refreshTokenHeader);

            if (StringUtils.isBlank(accessTokenHeader) || !accessTokenHeader.startsWith("Bearer ")) {
                throw new IllegalArgumentException("잘못된 Authorization 헤더 형식 현재 값: [" + accessTokenHeader + "]");
            }

            String paramAccessToken = TokenUtils.getHeaderToToken(accessTokenHeader);
            System.out.println("추출된 Access Token: " + paramAccessToken);

            // 블랙리스트 체크
            if (tokenBlackListService.isContainToken(paramAccessToken)) {
                throw new IllegalArgumentException("만료된 토큰입니다!");
            }

            ValidTokenDto accTokenValidDto = tokenUtils.isValidToken(paramAccessToken);
            if (accTokenValidDto.isValid()) {
                String userId = tokenUtils.getClaimsToUserId(paramAccessToken);
                Long userIdLong = Long.valueOf(userId);

                // DB 조회로 UserDto 구성
                User user = userRepository.findById(userIdLong)
                        .orElseThrow(() -> new RuntimeException("유저를 찾을 수 없습니다."));
                UserDto userDto = new UserDto(user);
                List<SimpleGrantedAuthority> authorities = userDto.getRoles().stream()
                    .map(SimpleGrantedAuthority::new)
                    .toList();
                UsernamePasswordAuthenticationToken authentication =
                    new UsernamePasswordAuthenticationToken(userDto, null, authorities);

                SecurityContextHolder.getContext().setAuthentication(authentication);
                chain.doFilter(request, response);  // 성공하면 필터 통과
            } else if ("TOKEN_EXPIRED".equals(accTokenValidDto.getErrorName())) {
                if (StringUtils.isNotBlank(refreshTokenHeader) && refreshTokenHeader.startsWith("Bearer ")) {
                    String paramRefreshToken = TokenUtils.getHeaderToToken(refreshTokenHeader);
                    ValidTokenDto refTokenValidDto = tokenUtils.isValidToken(paramRefreshToken);
                    if (refTokenValidDto.isValid()) {
                        UserDto claimsToUserDto = tokenUtils.getClaimsToUserDto(paramRefreshToken, false);
                        String token = tokenUtils.generateJwt(claimsToUserDto);
                        sendToClientAccessToken(token, response);
                        chain.doFilter(request, response);
                    } else {
                        throw new IllegalArgumentException("리프레시 토큰이 유효하지 않습니다. 다시 로그인이 필요합니다.");
                    }
                } else {
                    throw new IllegalArgumentException("리프레시 토큰이 없습니다. 다시 로그인이 필요합니다.");
                }
            } else {
                throw new IllegalArgumentException("토큰이 유효하지 않습니다.");
            }

        } catch (Exception e) {
            jwtTokenError(response, e);
        }
    }

    //인증이 필요 없는 경로인지 확인하는 메서드
    private boolean isWhitelisted(String uri) {
        return WHITELIST_PREFIXES.stream().anyMatch(uri::startsWith);
    }

    //JWT 관련 에러가 발생했을 때, JSON 형태로 에러 응답을 내려주는 메서드
    private void jwtTokenError(@NonNull HttpServletResponse response, Exception e) {
        ObjectMapper om = new ObjectMapper();
        Map<String, Object> resultMap = new HashMap<>();
        String resultMsg;

        if (e instanceof ExpiredJwtException) {
            resultMsg = "토큰 기간이 만료되었습니다.";
        } else if (e instanceof JwtException) {
            resultMsg = "잘못된 토큰이 발급되었습니다.";
        } else {
            resultMsg = "알 수 없는 토큰 오류가 발생했습니다.";
        }

        resultMap.put("status", 403);
        resultMap.put("code", "9999");
        resultMap.put("message", resultMsg);
        resultMap.put("reason", e.getMessage());

        response.setStatus(HttpServletResponse.SC_FORBIDDEN);
        response.setContentType("application/json;charset=UTF-8");

        try {
            String json = om.writeValueAsString(resultMap);
            response.getWriter().write(json);
        } catch (IOException ex) {
            log.error(" JSON 응답 처리 중 오류 발생: {}", ex.getMessage());
        }
    }


    //Access Token이 만료됐지만, 유효한 Refresh Token으로 재발급이 가능할 때 새로운 Access Token을 생성
    private void sendToClientAccessToken(String token, HttpServletResponse response) {
        Map<String, Object> resultMap = new HashMap<>();
        ObjectMapper om = new ObjectMapper();
        resultMap.put("status", 401);
        resultMap.put("failMsg", null);
        resultMap.put("accessToken", token);
        response.setCharacterEncoding("UTF-8");
        response.setContentType("application/json");
        try {
            PrintWriter printWriter = response.getWriter();
            printWriter.write(om.writeValueAsString(resultMap));
            printWriter.flush();
            printWriter.close();
        } catch (IOException e) {
            log.error("[-] 결과값 생성에 실패 : {}", e);
        }

    }
}
