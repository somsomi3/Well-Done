package com.be.domain.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.*;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

@Slf4j
@Component
public class UserSocketHandler extends TextWebSocketHandler {

    private final Map<String, Set<WebSocketSession>> rooms = new ConcurrentHashMap<>();
    private final ObjectMapper objectMapper = new ObjectMapper();

    @Override
    public void afterConnectionEstablished(WebSocketSession session) {
        log.info("사용자 WebSocket 연결됨: {}", session.getId());
    }

    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
        JsonNode json = objectMapper.readTree(message.getPayload());

        String type = json.has("type") ? json.get("type").asText() : "unknown";
        String roomId = json.has("roomId") ? json.get("roomId").asText() : null;

        if (roomId == null) {
            log.warn("roomId가 없어 메시지를 처리할 수 없습니다. sessionId: {}", session.getId());
            return;
        }

        rooms.computeIfAbsent(roomId, k -> ConcurrentHashMap.newKeySet()).add(session);

        switch (type) {
            case "command":
                broadcast(roomId, message, session);
                break;
            case "join":
                log.info("사용자 {} 방 {}에 참여", session.getId(), roomId);
                break;
            default:
                log.warn("알 수 없는 메시지 타입: {}", type);
        }
    }

    private void broadcast(String roomId, TextMessage message, WebSocketSession sender) {
        Set<WebSocketSession> sessions = rooms.get(roomId);
        if (sessions == null) {
            log.warn("방 {}는 존재하지 않습니다", roomId);
            return;
        }

        for (WebSocketSession session : sessions) {
            try {
                if (session.isOpen() && !session.getId().equals(sender.getId())) {
                    session.sendMessage(message);
                }
            } catch (Exception e) {
                log.error("메시지 전송 실패 - 세션: {}", session.getId(), e);
            }
        }
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) {
        rooms.values().forEach(set -> set.remove(session));
        log.info("사용자 WebSocket 연결 종료: {}", session.getId());
    }
}
