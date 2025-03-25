package com.be.domain.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.*;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

@Slf4j
@Component
public class SimulatorSocketHandler extends TextWebSocketHandler {

    private final ObjectMapper objectMapper = new ObjectMapper();

    //방 ID별 세션 관리
    private final Map<String, Set<WebSocketSession>> rooms = new ConcurrentHashMap<>();
    private final Map<String, String> sessionIdToRoomId = new ConcurrentHashMap<>();

    enum MessageType {
        POSITION, MAP, EVENT, JOIN, LEAVE, UNKNOWN;

        public static MessageType from(String type) {
            try {
                return MessageType.valueOf(type.toUpperCase());
            } catch (Exception e) {
                return UNKNOWN;
            }
        }
    }

    @Override
    public void afterConnectionEstablished(WebSocketSession session) {
        log.info("시뮬레이터 연결됨: {}", session.getId());
    }

    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
        JsonNode json = objectMapper.readTree(message.getPayload());

        String typeStr = json.has("type") ? json.get("type").asText() : "unknown";
        String roomId = json.has("roomId") ? json.get("roomId").asText() : null;

        if (roomId == null) {
            log.warn("roomId가 없습니다.");
            return;
        }

        // type 분기 처리
        MessageType type = MessageType.from(typeStr);

        switch (type) {
            case JOIN -> handleJoin(session, roomId);
            case POSITION, MAP, EVENT -> handleBroadcast(session, roomId, message);
            case LEAVE -> handleLeave(session, roomId);
            default -> log.warn("알 수 없는 메시지 타입: {}", typeStr);
        }
    }

    private void handleJoin(WebSocketSession session, String roomId) {
        rooms.computeIfAbsent(roomId, k -> ConcurrentHashMap.newKeySet()).add(session);
        sessionIdToRoomId.put(session.getId(), roomId);
        log.info("{} 가 방 {} 에 참여", session.getId(), roomId);
    }

    private void handleBroadcast(WebSocketSession sender, String roomId, TextMessage message) {
        Set<WebSocketSession> roomSessions = rooms.getOrDefault(roomId, Collections.emptySet());
        for (WebSocketSession session : roomSessions) {
            if (session.isOpen() && !session.getId().equals(sender.getId())) {
                try {
                    session.sendMessage(message);
                } catch (Exception e) {
                    log.error("메시지 전송 실패: {}", session.getId(), e);
                }
            }
        }
    }

    private void handleLeave(WebSocketSession session, String roomId) {
        Set<WebSocketSession> roomSessions = rooms.get(roomId);
        if (roomSessions != null) {
            roomSessions.remove(session);
            log.info("세션 {} 가 방 {} 에서 나감", session.getId(), roomId);
        }
        sessionIdToRoomId.remove(session.getId());
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) {
        String roomId = sessionIdToRoomId.get(session.getId());
        if (roomId != null) {
            handleLeave(session, roomId);
        }
        log.info("시뮬레이터 연결 종료: {}", session.getId());
    }
}

