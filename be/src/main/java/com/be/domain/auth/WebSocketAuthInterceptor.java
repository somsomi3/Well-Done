package com.be.domain.auth;

import com.be.domain.auth.utils.TokenUtils;
import com.be.domain.auth.utils.ValidTokenDto;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.server.ServerHttpRequest;
import org.springframework.http.server.ServerHttpResponse;
import org.springframework.http.server.ServletServerHttpRequest;
import org.springframework.http.server.ServletServerHttpResponse;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.WebSocketHandler;
import org.springframework.web.socket.server.HandshakeInterceptor;

import java.net.URI;
import java.util.Map;

@Slf4j
@Component
public class WebSocketAuthInterceptor implements HandshakeInterceptor {
    @Autowired
    private TokenUtils tokenUtils;
    @Override
    public boolean beforeHandshake(ServerHttpRequest request, ServerHttpResponse response,
                                   WebSocketHandler wsHandler, Map<String, Object> attributes) throws Exception {

        log.info("[WebSocketAuthInterceptor] beforeHandshake 호출됨");

        if (request instanceof ServletServerHttpRequest servletRequest &&
                response instanceof ServletServerHttpResponse servletResponse) {

            HttpServletRequest httpRequest = servletRequest.getServletRequest();
            HttpServletResponse httpResponse = servletResponse.getServletResponse();

            // 헤더에서 Authorization 토큰 얻기
            String headerToken = httpRequest.getHeader("Authorization");
            log.info("🔑 Header에서 추출된 토큰: {}", headerToken);

            // 쿼리에서 token 얻기
            URI uri = request.getURI();
            String queryToken = extractTokenFromQuery(uri);
            log.info("Query에서 추출된 토큰: {}", queryToken);

            String token = (headerToken != null) ? headerToken : queryToken;
            if (token == null || !isValidToken(token)) {
                log.warn("유효하지 않은 토큰! 연결 거부");
                httpResponse.setStatus(HttpServletResponse.SC_UNAUTHORIZED); // 401 Unauthorized
                return false;
            }

            log.info("토큰 유효성 검사 통과!");

            attributes.put("user", getUserFromToken(token));
            return true;
        }

        log.warn("요청이 Servlet 기반이 아닙니다. 연결 거부!");
        return false;
    }

    @Override
    public void afterHandshake(ServerHttpRequest request, ServerHttpResponse response,
                               WebSocketHandler wsHandler, Exception exception) {
        log.info("WebSocket 핸드셰이크 성공");
    }

    private boolean isValidToken(String token) {
        if (token == null || token.trim().isEmpty()) {
            log.warn("토큰이 비어있습니다.");
            return false;
        }

        ValidTokenDto tokenDto = tokenUtils.isValidToken(token);

        if (!tokenDto.isValid()) {
            log.warn("토큰 검증 실패: {}", tokenDto.getErrorName());
            return false;
        }

        log.info("JWT 토큰 검증 성공");
        return true;
    }

    private Object getUserFromToken(String token) {
        return tokenUtils.getClaimsToUserDto(token, true);
    }

    // 쿼리 파라미터에서 token을 추출하는 메소드
    private String extractTokenFromQuery(URI uri) {
        String query = uri.getQuery();
        if (query == null || !query.contains("token=")) {
            return null;
        }

        String[] params = query.split("&");
        for (String param : params) {
            if (param.startsWith("token=")) {
                return param.substring("token=".length());
            }
        }
        return null;
    }
}
