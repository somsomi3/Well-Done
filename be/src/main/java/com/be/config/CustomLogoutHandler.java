package com.be.config;

import com.be.domain.auth.service.TokenBlackListService;
import com.be.domain.auth.utils.TokenUtils;
import com.fasterxml.jackson.databind.ObjectMapper;

import jakarta.servlet.http.Cookie;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.security.core.Authentication;
import org.springframework.security.web.authentication.logout.LogoutHandler;
import org.springframework.stereotype.Component;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
@Slf4j
@Component
@RequiredArgsConstructor
public class CustomLogoutHandler implements LogoutHandler {

    private final TokenBlackListService tokenBlackListService;
    private final TokenUtils tokenUtils;

    @Override
    public void logout(HttpServletRequest request, HttpServletResponse response, Authentication authentication) {
        log.info("[LOGOUT] 요청 수신");

        String headerToken = request.getHeader("Authorization");
        if (headerToken == null || headerToken.isEmpty()) {
            log.warn("[LOGOUT] Authorization 헤더 없음");
            sendErrorResponse(response, "토큰이 존재하지 않습니다.");
            return;
        }

        String token = TokenUtils.getHeaderToToken(headerToken);
        log.debug("[LOGOUT] 토큰 추출: {}", token);

        if (!tokenUtils.isValidToken(token).isValid()) {
            log.warn("[LOGOUT] 유효하지 않은 토큰");
            sendErrorResponse(response, "잘못된 토큰입니다.");
            return;
        }

        if (tokenBlackListService.isContainToken(token)) {
            log.warn("[LOGOUT] 이미 블랙리스트에 있는 토큰");
            sendErrorResponse(response, "이미 로그아웃된 토큰입니다.");
            return;
        }

        tokenBlackListService.addTokenToList(token);
        log.info("[LOGOUT] 토큰 블랙리스트에 추가 완료");

        deleteRefreshToken(response);
        log.info("[LOGOUT] Refresh Token 삭제 완료");

        response.setStatus(HttpServletResponse.SC_OK);
        log.info("[LOGOUT] 로그아웃 처리 완료");
    }

    private void sendErrorResponse(HttpServletResponse response, String message) {
        response.setStatus(HttpServletResponse.SC_UNAUTHORIZED);
        response.setContentType("application/json");

        Map<String, Object> resultMap = new HashMap<>();
        resultMap.put("resultCode", 9999);
        resultMap.put("failMsg", message);

        try {
            response.getWriter().write(new ObjectMapper().writeValueAsString(resultMap));
        } catch (IOException e) {
            log.error("[LOGOUT] 에러 응답 전송 실패", e);
        }
    }

    private void deleteRefreshToken(HttpServletResponse response) {
        Cookie refreshTokenCookie = new Cookie("refreshToken", null);
        refreshTokenCookie.setHttpOnly(true);
        refreshTokenCookie.setSecure(true);
        refreshTokenCookie.setPath("/");
        refreshTokenCookie.setMaxAge(0);
        response.addCookie(refreshTokenCookie);
    }
}
