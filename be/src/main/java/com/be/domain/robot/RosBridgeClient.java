package com.be.domain.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.*;
import org.springframework.web.socket.client.standard.StandardWebSocketClient;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import jakarta.annotation.PostConstruct; // jakarta로 변경
import java.net.URI;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

@Slf4j
@Component
public class RosBridgeClient {

    private final StandardWebSocketClient client;
    private WebSocketSession session;
    private final ObjectMapper objectMapper;
    private final Map<String, Consumer<JsonNode>> topicCallbacks;
    private final UserSocketHandler userSocketHandler;

    private static final String ROS_BRIDGE_URI = "ws://localhost:9090"; // ROS Bridge 주소
    private static final int RECONNECT_DELAY_MS = 5000; // 재연결 시도 간격 (5초)
    private boolean reconnectEnabled = true;

    @Autowired
    public RosBridgeClient(UserSocketHandler userSocketHandler) {
        this.userSocketHandler = userSocketHandler;
        this.client = new StandardWebSocketClient();
        this.objectMapper = new ObjectMapper();
        this.topicCallbacks = new ConcurrentHashMap<>();
    }

    @PostConstruct
    public void init() {
        // 빈 초기화 후 ROS Bridge 연결 시작

    }

    /**
     * ROS Bridge에 연결
     */
    public void connect() {
        try {
            CompletableFuture<WebSocketSession> futureSession = client.execute(
                    new RosBridgeHandler(), ROS_BRIDGE_URI);

            // 5초 안에 연결이 안되면 타임아웃
            this.session = futureSession.get(5, TimeUnit.SECONDS);
            log.info("ROS Bridge에 연결되었습니다.");
        } catch (Exception e) {
            log.error("ROS Bridge 연결 실패", e);
            scheduleReconnect();
        }
    }

    /**
     * ROS 토픽 구독
     * @param topic ROS 토픽 이름
     * @param messageType ROS 메시지 타입 (예: "std_msgs/String")
     * @param callback 메시지 수신 시 호출할 콜백 함수
     */
    public void subscribe(String topic, String messageType, Consumer<JsonNode> callback) {
        if (session == null || !session.isOpen()) {
            log.warn("ROS Bridge 연결이 없습니다. 구독 요청 무시됨: {}", topic);
            return;
        }

        try {
            ObjectNode message = objectMapper.createObjectNode();
            message.put("op", "subscribe");
            message.put("topic", topic);
            message.put("type", messageType);

            session.sendMessage(new TextMessage(message.toString()));
            topicCallbacks.put(topic, callback);
            log.info("ROS 토픽 구독 시작: {}", topic);
        } catch (Exception e) {
            log.error("ROS 토픽 구독 실패: {}", topic, e);
        }
    }

    /**
     * ROS 토픽에 메시지 발행
     * @param topic ROS 토픽 이름
     * @param messageType ROS 메시지 타입
     * @param message ROS 메시지 내용
     */
    public void publish(String topic, String messageType, Map<String, Object> message) {
        if (session == null || !session.isOpen()) {
            log.warn("ROS Bridge 연결이 없습니다. 발행 요청 무시됨: {}", topic);
            return;
        }

        try {
            Map<String, Object> rosMessage = new HashMap<>();
            rosMessage.put("op", "publish");
            rosMessage.put("topic", topic);
            rosMessage.put("type", messageType);
            rosMessage.put("msg", message);

            String jsonMessage = objectMapper.writeValueAsString(rosMessage);
            session.sendMessage(new TextMessage(jsonMessage));
            log.debug("ROS 메시지 발행: {}", topic);
        } catch (Exception e) {
            log.error("ROS 메시지 발행 실패: {}", topic, e);
        }
    }

    /**
     * ROS 서비스 호출
     * @param service ROS 서비스 이름
     * @param args 서비스 호출 인자
     * @param callback 응답 처리 콜백
     */
    public void callService(String service, Map<String, Object> args, Consumer<JsonNode> callback) {
        if (session == null || !session.isOpen()) {
            log.warn("ROS Bridge 연결이 없습니다. 서비스 호출 무시됨: {}", service);
            return;
        }

        try {
            String serviceCallId = "call_" + System.currentTimeMillis();

            Map<String, Object> rosMessage = new HashMap<>();
            rosMessage.put("op", "call_service");
            rosMessage.put("service", service);
            rosMessage.put("args", args);
            rosMessage.put("id", serviceCallId);

            String jsonMessage = objectMapper.writeValueAsString(rosMessage);
            session.sendMessage(new TextMessage(jsonMessage));

            // 서비스 응답 콜백 등록
            topicCallbacks.put("service_response:" + serviceCallId, callback);
            log.info("ROS 서비스 호출: {}", service);
        } catch (Exception e) {
            log.error("ROS 서비스 호출 실패: {}", service, e);
        }
    }

    /**
     * 연결 끊김 시 재연결 스케줄링
     */
    private void scheduleReconnect() {
        if (reconnectEnabled) {
            log.info("{}ms 후 ROS Bridge 재연결 시도...", RECONNECT_DELAY_MS);

            new Thread(() -> {
                try {
                    Thread.sleep(RECONNECT_DELAY_MS);
                    connect();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }
    }

    /**
     * ROS Bridge WebSocket 핸들러
     */
    private class RosBridgeHandler extends TextWebSocketHandler {

        @Override
        public void afterConnectionEstablished(WebSocketSession session) {
            log.info("ROS Bridge 연결 성공: {}", session.getId());
        }

        @Override
        protected void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
            String payload = message.getPayload();
            JsonNode jsonNode = objectMapper.readTree(payload);

            // op 필드 확인
            if (jsonNode.has("op")) {
                String op = jsonNode.get("op").asText();

                if ("publish".equals(op) && jsonNode.has("topic")) {
                    // 토픽 메시지 처리
                    String topic = jsonNode.get("topic").asText();
                    if (topicCallbacks.containsKey(topic)) {
                        topicCallbacks.get(topic).accept(jsonNode.get("msg"));
                    }

                    // 사용자에게 ROS 메시지 전달
                    forwardToUsers(topic, jsonNode.get("msg"));
                } else if ("service_response".equals(op) && jsonNode.has("id")) {
                    // 서비스 응답 처리
                    String id = jsonNode.get("id").asText();
                    String callbackKey = "service_response:" + id;
                    if (topicCallbacks.containsKey(callbackKey)) {
                        topicCallbacks.get(callbackKey).accept(jsonNode.get("values"));
                    }
                }
            }
        }

        /**
         * ROS 메시지를 사용자 소켓으로 전달
         */
        private void forwardToUsers(String topic, JsonNode msg) {
            try {
                // 사용자 소켓으로 전달할 메시지 포맷 구성
                ObjectNode userMessage = objectMapper.createObjectNode();
                userMessage.put("type", "ros_message");
                userMessage.put("topic", topic);
                userMessage.set("data", msg);

                // 클라이언트에 브로드캐스트 (TextMessage를 직접 전달)
                TextMessage textMessage = new TextMessage(userMessage.toString());

                // 실제 UserSocketHandler의 메서드를 통해 전달
                // 브로드캐스트 메서드가 없다면 다른 방법으로 구현 필요
                try {
                    // 방법 1: UserSocketHandler 내부에 broadcastAll 메서드가 있다면
                    // userSocketHandler.broadcastAll(textMessage);
                } catch (Exception e) {
                    log.error("사용자에게 ROS 메시지 전달 실패", e);
                }
            } catch (Exception e) {
                log.error("사용자에게 ROS 메시지 전달 오류", e);
            }
        }

        @Override
        public void handleTransportError(WebSocketSession session, Throwable exception) {
            log.error("ROS Bridge 연결 오류", exception);
        }

        @Override
        public void afterConnectionClosed(WebSocketSession session, CloseStatus status) {
            log.warn("ROS Bridge 연결 종료: {}", status);
            scheduleReconnect();
        }
    }
}