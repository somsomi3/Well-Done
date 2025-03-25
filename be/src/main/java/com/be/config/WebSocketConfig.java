package com.be.config;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;

@RequiredArgsConstructor
@Slf4j
@Configuration
@EnableWebSocket
public class WebSocketConfig implements WebSocketConfigurer {

//    private final SimulatorSocketHandler simulatorSocketHandler;
//    private final UserSocketHandler userSocketHandler;

    @Override
    public void registerWebSocketHandlers(WebSocketHandlerRegistry registry) {
//        registry.addHandler(simulatorSocketHandler, "/ws/simulator").setAllowedOrigins("*");
//        registry.addHandler(userSocketHandler, "/ws/user").setAllowedOrigins("*");
    }


}