package com.be.config;

import com.be.domain.auth.WebSocketAuthInterceptor;
import com.be.domain.robot.SimulatorSocketHandler;
import com.be.domain.robot.UserSocketHandler;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;
import org.springframework.web.socket.server.support.WebSocketHttpRequestHandler;
import org.springframework.web.socket.server.HandshakeInterceptor;

@RequiredArgsConstructor
@Slf4j
@Configuration
@EnableWebSocket
public class WebSocketConfig implements WebSocketConfigurer {

    private final SimulatorSocketHandler simulatorSocketHandler;
    private final UserSocketHandler userSocketHandler;

    // 인증을 위한 인터셉터 추가
    private final WebSocketAuthInterceptor webSocketAuthInterceptor;

    @Override
    public void registerWebSocketHandlers(WebSocketHandlerRegistry registry) {
        log.info("WebSocket Handlers 등록");

        registry.addHandler(simulatorSocketHandler, "/ws/simulator")
                .setAllowedOrigins("*")
                .addInterceptors(webSocketAuthInterceptor);

        log.info("simulatorSocketHandler 등록 (path: /ws/simulator)");

        registry.addHandler(userSocketHandler, "/ws/user")
                .setAllowedOrigins("*")
                .addInterceptors(webSocketAuthInterceptor);

        log.info("userSocketHandler 등록(path: /ws/user)");
    }

}
