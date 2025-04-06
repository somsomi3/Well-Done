package com.be.domain.robot;

import com.be.domain.robot.service.RobotService;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.annotation.Lazy;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.*;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

@Slf4j
@Component
public class UserSocketHandler extends TextWebSocketHandler {

    // 전역적으로 모든 세션 관리
    private final Set<WebSocketSession> sessions = ConcurrentHashMap.newKeySet();
    private final ObjectMapper objectMapper = new ObjectMapper();

    // RobotService는 순환 참조를 피하기 위해 지연 주입
    @Autowired
    @Lazy
    private RobotService robotService;

    @Override
    public void afterConnectionEstablished(WebSocketSession session) {
        sessions.add(session);
        log.info("사용자 WebSocket 연결됨: {}", session.getId());
    }

    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
        JsonNode json = objectMapper.readTree(message.getPayload());
        String type = json.has("type") ? json.get("type").asText() : "unknown";

        switch (type) {
            case "command":
                // ROS 명령인지 확인
                if (json.has("robot") && json.get("robot").asBoolean()) {
                    // RobotService로 전달
                    robotService.handleUserCommand(json);
                } else {
                    // 일반 메시지는 기존 로직대로 처리
                    broadcast(message, session);
                }
                break;
            case "offer":
            case "answer":
            case "candidate":
            case "ping":
                broadcast(message, session);
                break;

            case "join":
                log.info("사용자 {}가 입장하였습니다.", session.getId());
                break;

            default:
                log.warn("알 수 없는 메시지 타입: {}", type);
        }
    }

    private void broadcast(TextMessage message, WebSocketSession sender) {
        sessions.stream()
                .filter(session -> session.isOpen() && !session.getId().equals(sender.getId()))
                .forEach(session -> {
                    try {
                        session.sendMessage(message);
                    } catch (Exception e) {
                        log.error("메시지 전송 실패 - 세션: {}", session.getId(), e);
                    }
                });
    }

    public void broadcastAll(TextMessage message) {
        sessions.stream()
                .filter(WebSocketSession::isOpen)
                .forEach(session -> {
                    try {
                        session.sendMessage(message);
                    } catch (Exception e) {
                        log.error("메시지 전송 실패 - 세션: {}", session.getId(), e);
                    }
                });
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) {
        sessions.remove(session);
        log.info("사용자 WebSocket 연결 종료: {}", session.getId());
    }


    //맵데이터를 실시간으로 WebSocket으로 브로드캐스트하기
    public void broadcastMap(Object mapData) {
        try {
            String message = objectMapper.writeValueAsString(mapData);
            broadcastAll(new TextMessage(message));
        } catch (Exception e) {
            log.error("맵 데이터 전송 실패", e);
        }
    }
}

