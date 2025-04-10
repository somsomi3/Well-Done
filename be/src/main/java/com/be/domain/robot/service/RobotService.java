package com.be.domain.robot.service;

import com.be.domain.robot.RosBridgeClient;
import com.be.domain.robot.UserSocketHandler;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.web.socket.TextMessage;

import jakarta.annotation.PostConstruct; // jakarta로 변경
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@RequiredArgsConstructor
@Slf4j
@Service
public class RobotService {

    private final RosBridgeClient rosBridgeClient;
    private final UserSocketHandler userSocketHandler;
    private final ObjectMapper objectMapper;
    private final RedisService redisService;

    // 최신 카메라 이미지 저장 변수
    @Getter
    private Map<String, Object> latestCameraImage;

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

    /**
     * 로봇 현재 위치 전송
     */

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

    /**
     * 압축된 JPEG 카메라 이미지 저장 및 클라이언트로 전송
     * @param imageData 압축된 이미지 데이터
     */

    public void processCameraImage(Map<String, Object> imageData) {
        // 최신 이미지 저장
        this.latestCameraImage = imageData;

        try {
            // WebSocket으로 전송하기 위한 데이터 포맷 구성
            Map<String, Object> socketData = new HashMap<>();
            socketData.put("type", "camera_image");
            socketData.put("data", imageData.get("data")); // Base64 인코딩된 이미지 데이터
            socketData.put("timestamp", System.currentTimeMillis());

            // ObjectMapper로 JSON 변환
            String json = objectMapper.writeValueAsString(socketData);

            // Redis 저장 시도 (선택 사항)
            try {
                redisService.saveRobotCameraImage(json);
                log.debug("Redis에 카메라 이미지 저장 완료");
            } catch (Exception redisError) {
                log.error("Redis 저장 실패 (무시하고 계속 진행): {}", redisError.getMessage());
            }

            // WebSocket으로 전송
            userSocketHandler.broadcastAll(new TextMessage(json));
            log.debug("WebSocket으로 카메라 이미지 전송 완료: {} bytes", ((String) imageData.get("data")).length());

        } catch (Exception e) {
            log.error("카메라 이미지 처리 중 오류 발생", e);
        }
    }

    /**
     * 맵핑 완료 데이터 처리 및 저장
     * @param mappingDoneData 맵핑 완료 데이터
     */
    public void processMappingDoneData(Map<String, Object> mappingDoneData) {
        try {
            // 1. 원본 데이터 JSON 변환 및 저장
            String jsonOriginal = objectMapper.writeValueAsString(mappingDoneData);
            redisService.saveMappingDoneData(jsonOriginal);
            log.info("Redis에 원본 맵핑 완료 데이터 저장 완료");

            // 2. 맵 데이터 처리 및 저장
            boolean success = (boolean) mappingDoneData.get("success");
            Map<String, Object> notification = new HashMap<>();
            notification.put("type", "mapping_complete");
            notification.put("success", success);
            notification.put("timestamp", System.currentTimeMillis());

            if (success) {
                // 일반 맵 처리
                Map<String, Object> mapData = (Map<String, Object>) mappingDoneData.get("map");
                processAndSaveMapData(mapData, false);

                // 인플레이트된 맵 처리
                Map<String, Object> inflatedMapData = (Map<String, Object>) mappingDoneData.get("map_inflated");
                processAndSaveMapData(inflatedMapData, true);
            }
            // WebSocket으로 알림 전송
            try {
                String json = objectMapper.writeValueAsString(notification);
                userSocketHandler.broadcastAll(new TextMessage(json));
                log.info("매핑 완료 알림 전송: success={}", success);
            } catch (Exception e) {
                log.error("매핑 완료 알림 전송 중 오류 발생", e);
            }

        } catch (Exception e) {
            log.error("맵핑 완료 데이터 처리 중 오류 발생", e);
        }
    }

    /**
     * 전시 완료 데이터 처리 및 저장
     * @param placeDoneData 전시 완료 데이터
     */
    public void processPlaceDoneData(Map<String, Object> placeDoneData) {
        try {
            boolean success = (boolean) placeDoneData.get("success");
            String productId = (String) placeDoneData.get("product_id");
            String toId = (String) placeDoneData.get("to_id");

            // 알림 데이터 준비
            Map<String, Object> notification = new HashMap<>();
            notification.put("type", "place_complete");
            notification.put("success", success);
            notification.put("product_id", productId);
            notification.put("to_id", toId);
            notification.put("timestamp", System.currentTimeMillis());

            if (success && placeDoneData.containsKey("map") && placeDoneData.containsKey("map_inflated")) {
                // 일반 맵 처리
                Map<String, Object> mapData = (Map<String, Object>) placeDoneData.get("map");
                processAndSaveMapData(mapData, false);

                // 인플레이트된 맵 처리
                Map<String, Object> inflatedMapData = (Map<String, Object>) placeDoneData.get("map_inflated");
                processAndSaveMapData(inflatedMapData, true);

                log.info("전시 완료 후 맵 데이터 처리 완료");
            }

            // WebSocket으로 알림 전송
            try {
                userSocketHandler.broadcastMap(notification);
                log.info("전시 완료 알림 전송: product_id={}, to_id={}", productId, toId);
            } catch (Exception e) {
                log.error("전시 완료 알림 전송 중 오류 발생", e);
            }

        } catch (Exception e) {
            log.error("전시 완료 데이터 처리 중 오류 발생", e);
        }
    }

    /**
     * 물건 집기 완료 데이터 처리 및 저장
     * @param pickDoneData 물건 집기 완료 데이터
     */
    public void processPickDoneData(Map<String, Object> pickDoneData) {
        try {
            boolean success = (boolean) pickDoneData.get("success");
            String productId = (String) pickDoneData.get("product_id");
            String fromId = (String) pickDoneData.get("from_id");

            // 알림 데이터 준비
            Map<String, Object> notification = new HashMap<>();
            notification.put("type", "pick_complete");
            notification.put("success", success);
            notification.put("product_id", productId);
            notification.put("from_id", fromId);
            notification.put("timestamp", System.currentTimeMillis());

            if (success && pickDoneData.containsKey("map") && pickDoneData.containsKey("map_inflated")) {
                // 일반 맵 처리
                Map<String, Object> mapData = (Map<String, Object>) pickDoneData.get("map");
                processAndSaveMapData(mapData, false);

                // 인플레이트된 맵 처리
                Map<String, Object> inflatedMapData = (Map<String, Object>) pickDoneData.get("map_inflated");
                processAndSaveMapData(inflatedMapData, true);

                log.info("물건 집기 완료 후 맵 데이터 처리 완료");
            }

            // WebSocket으로 알림 전송
            try {
                userSocketHandler.broadcastMap(notification);
                log.info("물건 집기 완료 알림 전송: product_id={}, from_id={}", productId, fromId);
            } catch (Exception e) {
                log.error("물건 집기 완료 알림 전송 중 오류 발생", e);
            }

        } catch (Exception e) {
            log.error("물건 집기 완료 데이터 처리 중 오류 발생", e);
        }
    }

    /**
     * 맵 데이터 처리 및 저장
     * @param mapData 맵 데이터
     * @param isInflated 인플레이트된 맵 여부
     */
    private void processAndSaveMapData(Map<String, Object> mapData, boolean isInflated) {
        try {
            // 1. 맵 정보 추출
            Map<String, Object> info = (Map<String, Object>) mapData.get("info");
            int width = ((Number) info.get("width")).intValue();
            int height = ((Number) info.get("height")).intValue();
            double resolution = ((Number) info.get("resolution")).doubleValue();

            // 2. 데이터 배열 처리
            List<Number> rawData = (List<Number>) mapData.get("data");
            int[][] processedMap = new int[height][width];

            // 3. 데이터 변환 (1D → 2D 배열)
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    int index = y * width + x;
                    if (index < rawData.size()) {
                        processedMap[y][x] = rawData.get(index).intValue();
                    } else {
                        processedMap[y][x] = -1; // 기본값 설정
                    }
                }
            }

            // 4. 전송용 객체 구성
            Map<String, Object> payload = new HashMap<>();
            payload.put("type", isInflated ? "map_inflated" : "map");
            payload.put("width", width);
            payload.put("height", height);
            payload.put("resolution", resolution);
            payload.put("map", processedMap);

            // 원점 정보 추가
            Map<String, Object> origin = (Map<String, Object>) info.get("origin");
            payload.put("origin", origin);

            // 5. JSON 변환 및 Redis에 저장
            String json = objectMapper.writeValueAsString(payload);
            redisService.saveMapData(json, isInflated);
            log.info("Redis에 처리된 맵 데이터 저장 완료: isInflated={}", isInflated);

            // 6. WebSocket으로 실시간 전송 (기본 맵만)
            if (!isInflated) {
                userSocketHandler.broadcastMap(payload);
            }
        } catch (Exception e) {
            log.error("맵 데이터 처리 및 저장 중 오류 발생", e);
        }
    }

}