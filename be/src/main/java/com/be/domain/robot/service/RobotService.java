package com.be.domain.robot.service;

import com.be.domain.robot.RosBridgeClient;
import com.be.domain.robot.UserSocketHandler;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.web.socket.TextMessage;

import jakarta.annotation.PostConstruct; // jakarta로 변경
import java.util.HashMap;
import java.util.Map;

@RequiredArgsConstructor
@Slf4j
@Service
public class RobotService {

    private final RosBridgeClient rosBridgeClient;
    private final UserSocketHandler userSocketHandler;
    private final ObjectMapper objectMapper;
    private final RedisService redisService;



    @PostConstruct
    private void initializeRosTopics() {
        // 기본 ROS 토픽 구독 설정
        subscribeToRobotStatus();
        subscribeToRobotSensors();
    }

    /**
     * 로봇 상태 토픽 구독
     */
    private void subscribeToRobotStatus() {
        rosBridgeClient.subscribe("/robot/status", "std_msgs/String", (message) -> {
            log.info("로봇 상태 업데이트: {}", message);
            // 필요한 상태 처리 로직
        });
    }

    /**
     * 로봇 센서 데이터 토픽 구독
     */
    private void subscribeToRobotSensors() {
        rosBridgeClient.subscribe("/robot/sensors", "sensor_msgs/LaserScan", (message) -> {
            log.debug("로봇 센서 데이터 업데이트");
            // 센서 데이터 처리 로직
        });
    }

    /**
     * 로봇 이동 명령 전송
     * @param direction 이동 방향 (forward, backward, left, right, stop)
     * @param speed 이동 속도 (0.0 ~ 1.0)
     */
    public void moveRobot(String direction, double speed) {
        Map<String, Object> message = new HashMap<>();
        message.put("linear", new HashMap<String, Double>() {{
            put("x", "forward".equals(direction) ? speed : "backward".equals(direction) ? -speed : 0.0);
            put("y", 0.0);
            put("z", 0.0);
        }});
        message.put("angular", new HashMap<String, Double>() {{
            put("x", 0.0);
            put("y", 0.0);
            put("z", "left".equals(direction) ? speed : "right".equals(direction) ? -speed : 0.0);
        }});

        rosBridgeClient.publish("/cmd_vel", "geometry_msgs/Twist", message);
        log.info("로봇 이동 명령 전송: {}, 속도: {}", direction, speed);
    }

    /**
     * 로봇 작업 실행
     * @param taskType 작업 유형
     * @param params 작업 매개변수
     */
    public void executeTask(String taskType, Map<String, Object> params) {
        // 작업 타입에 따라 적절한 ROS 서비스 호출
        switch (taskType) {
            case "navigate":
                if (params.containsKey("x") && params.containsKey("y")) {
                    navigateToPosition((Double) params.get("x"), (Double) params.get("y"));
                }
                break;
            case "grab":
                grabObject(params);
                break;
            case "camera":
                adjustCamera(params);
                break;
            default:
                log.warn("지원되지 않는 작업 유형: {}", taskType);
        }
    }

    /**
     * 로봇 네비게이션 서비스 호출
     */
    private void navigateToPosition(double x, double y) {
        Map<String, Object> args = new HashMap<>();
        args.put("x", x);
        args.put("y", y);

        rosBridgeClient.callService("/navigate_to", args, (response) -> {
            log.info("네비게이션 서비스 응답: {}", response);
            // 필요한 응답 처리 로직
        });
    }

    /**
     * 물체 그랩 서비스 호출
     */
    private void grabObject(Map<String, Object> params) {
        rosBridgeClient.callService("/robot/grab", params, (response) -> {
            log.info("그랩 서비스 응답: {}", response);
            // 필요한 응답 처리 로직
        });
    }

    /**
     * 카메라 조정 메시지 발행
     */
    private void adjustCamera(Map<String, Object> params) {
        rosBridgeClient.publish("/robot/camera/control", "std_msgs/String", params);
        log.info("카메라 조정 명령 전송");
    }

    /**
     * WebSocket으로부터 받은 명령 처리
     * UserSocketHandler에서 호출될 메서드
     */
    public void handleUserCommand(JsonNode commandJson) {
        try {
            if (commandJson.has("action")) {
                String action = commandJson.get("action").asText();

                switch (action) {
                    case "move":
                        String direction = commandJson.get("direction").asText();
                        double speed = commandJson.has("speed") ?
                                commandJson.get("speed").asDouble() : 0.5;
                        moveRobot(direction, speed);
                        break;

                    case "task":
                        String taskType = commandJson.get("type").asText();
                        Map<String, Object> params = objectMapper.convertValue(
                                commandJson.get("params"), Map.class);
                        executeTask(taskType, params);
                        break;

                    default:
                        log.warn("지원되지 않는 액션: {}", action);
                }
            }
        } catch (Exception e) {
            log.error("명령 처리 중 오류 발생", e);
        }
    }

    public void sendCurrentPosition(String roomId, double x, double y) {
        // 위치 데이터에 type 필드 추가
        Map<String, Object> positionData = new HashMap<>();
        positionData.put("type", "position");
        positionData.put("x", x);
        positionData.put("y", y);

        try {
            // ObjectMapper로 JSON 변환
            String json = objectMapper.writeValueAsString(positionData);

            // Redis 저장 시도 (오류가 발생해도 계속 진행)
            try {
                redisService.saveRobotPosition(roomId, json);
                log.info("Redis에 좌표 저장 완료: {}", json);
            } catch (Exception redisError) {
                log.error("Redis 저장 실패 (무시하고 계속 진행): {}", redisError.getMessage());
            }

            // WebSocket으로 전송 (Redis 오류와 관계없이 실행됨)
            userSocketHandler.broadcastAll(new TextMessage(json));
            log.info("WebSocket으로 좌표 전송 완료: {}", json);

        } catch (Exception e) {
            log.error("좌표 처리 중 심각한 오류 발생", e);
        }
    }
}