package com.be.config;

import com.be.domain.auth.WebSocketAuthInterceptor;
import com.be.domain.robot.SimulatorSocketHandler;
import com.be.domain.robot.UserSocketHandler;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.context.annotation.Configuration;
import org.springframework.messaging.simp.config.MessageBrokerRegistry;
import org.springframework.web.socket.config.annotation.*;

@Slf4j
@RequiredArgsConstructor
@Configuration
@EnableWebSocket
@EnableWebSocketMessageBroker  // ✅ STOMP 메시징 활성화!
public class WebSocketConfig implements WebSocketConfigurer, WebSocketMessageBrokerConfigurer {

    private final SimulatorSocketHandler simulatorSocketHandler;
    private final UserSocketHandler userSocketHandler;
    private final WebSocketAuthInterceptor webSocketAuthInterceptor;

    /**
     * Low-Level WebSocketHandler 등록
     */
    @Override
    public void registerWebSocketHandlers(WebSocketHandlerRegistry registry) {
        log.info("WebSocket Handlers 등록");

        registry.addHandler(simulatorSocketHandler, "/ws/simulator")
                .setAllowedOrigins("*")
                .addInterceptors(webSocketAuthInterceptor);

        registry.addHandler(userSocketHandler, "/ws/user")
                .setAllowedOrigins("*")
                .addInterceptors(webSocketAuthInterceptor);
    }

    /**
     * STOMP 메시지 브로커 구성
     */
    @Override
    public void configureMessageBroker(MessageBrokerRegistry config) {
        config.enableSimpleBroker("/topic"); // 프론트가 구독할 경로
        config.setApplicationDestinationPrefixes("/app"); // 프론트가 메시지 보낼 때 prefix
    }

    /**
     * STOMP 엔드포인트 등록
     */
    @Override
    public void registerStompEndpoints(StompEndpointRegistry registry) {
        registry.addEndpoint("/ws-stomp") // 프론트에서 연결할 엔드포인트
                .setAllowedOriginPatterns("*")
                .withSockJS();
    }
}
