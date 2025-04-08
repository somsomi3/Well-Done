package com.be.domain.robot.controller;

import com.be.domain.robot.UserSocketHandler;
import com.be.domain.robot.service.RobotService;
import com.be.domain.storage.service.StorageService;
import lombok.RequiredArgsConstructor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.client.RestTemplate;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.List;

@RequiredArgsConstructor
@RestController
@RequestMapping("/api/robot")
public class RobotController {

    private final UserSocketHandler userSocketHandler;
    private final Logger log = LoggerFactory.getLogger(RobotController.class);
    private final RestTemplate restTemplate;
    private final StorageService storageService;

    // 브릿지 서버 URL 설정
    private final String bridgeUrl = "http://10.0.0.2:5000";
//    private final String bridgeUrl = "http://localhost:5000";

    // 최신 데이터 저장용 변수들
    private Map<String, Object> latestGlobalPath = new HashMap<>();
    private Map<String, Object> latestLocalPath = new HashMap<>();
    private Map<String, Object> latestOdometry = new HashMap<>();
    private Map<String, Object> latestScan = new HashMap<>();
    private Map<String, Object> latestMap = new HashMap<>();

    private final RobotService robotService;

    // 맵핑 완료 데이터 저장용 변수
    private Map<String, Object> latestMappingDoneResult = new HashMap<>();

    // 맵 상태 저장용 변수
    private Map<String, Object> latestMapStatus = new HashMap<>();

    // 장애물 감지 데이터 저장용 변수
    private Map<String, Object> latestObstacleAlert = new HashMap<>();

    // 목표 도달 상태 데이터 저장용 변수
    private Map<String, Object> latestGoalStatus = new HashMap<>();

    // 물건 집기 완료 데이터 저장용 변수
    private Map<String, Object> latestPickDone = new HashMap<>();

    // 전시 완료 데이터 저장용 변수
    private Map<String, Object> latestPlaceDone = new HashMap<>();

    // 압축된 JPEG 이미지 데이터를 위한 변수
    private Map<String, Object> latestCompressedImage = new HashMap<>();

    @PostMapping("/envir-status")
    public ResponseEntity<?> receiveEnvirStatus(@RequestBody Map<String, Object> data) {
        // 환경 상태 데이터 추출
        Integer month = ((Number) data.get("month")).intValue();
        Integer day = ((Number) data.get("day")).intValue();
        Integer hour = ((Number) data.get("hour")).intValue();
        Integer minute = ((Number) data.get("minute")).intValue();
        Integer temperature = ((Number) data.get("temperature")).intValue();
        String weather = (String) data.get("weather");

        // 데이터 로깅
//        log.info("환경 상태 수신: month={}, day={}, hour={}, minute={}, temperature={}, weather={}",
//                month, day, hour, minute, temperature, weather);

        // 필요한 경우 데이터베이스에 저장
        // ...

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "환경 상태 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/turtlebot-status")
    public ResponseEntity<?> receiveTurtlebotStatus(@RequestBody Map<String, Object> data) {
        // 터틀봇 상태 데이터 처리
//        log.info("터틀봇 상태 수신: {}", data);

        // 데이터 처리 로직
        // 예: 데이터베이스에 저장, 실시간 상태 업데이트 등
        // data안에 twist 데이터가 있는데 이게 아마 좌표값일듯???

        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "터틀봇 상태 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/move")
    public ResponseEntity<?> moveRobot(@RequestBody Map<String, Object> command) {
        // 이동 명령 추출
        Double linearX = ((Number) command.getOrDefault("linear_x", 0.0)).doubleValue();
        Double angularZ = ((Number) command.getOrDefault("angular_z", 0.0)).doubleValue();

        log.info("로봇 이동 명령: linear_x={}, angular_z={}", linearX, angularZ);

        // 브릿지 서버로 명령 전송
        Map<String, Object> requestData = new HashMap<>();
        requestData.put("command", "move");
        requestData.put("linear_x", linearX);
        requestData.put("angular_z", angularZ);

        try {
            // 주입된 RestTemplate 사용 및 멤버 변수 bridgeUrl 사용
            ResponseEntity<Map> response = this.restTemplate.postForEntity(
                    this.bridgeUrl + "/command", requestData, Map.class);

            return ResponseEntity.ok(response.getBody());
        } catch (Exception e) {
            log.error("로봇 명령 전송 실패: {}", e.getMessage());
            Map<String, Object> error = new HashMap<>();
            error.put("status", "error");
            error.put("message", "로봇에 명령을 전송하는 데 실패했습니다: " + e.getMessage());
            return ResponseEntity.status(500).body(error);
        }
    }

    @PostMapping("/global-path")
    public ResponseEntity<?> receiveGlobalPath(@RequestBody Map<String, Object> data) {
        // 데이터 로깅 (필요시 주석 해제)
        // log.info("글로벌 경로 데이터 수신: {}", data);

        // 최신 데이터 저장
        this.latestGlobalPath = data;

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "글로벌 경로 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/local-path")
    public ResponseEntity<?> receiveLocalPath(@RequestBody Map<String, Object> data) {
        // 데이터 로깅 (필요시 주석 해제)
         log.info("로컬 경로 데이터 수신: {}", data);

        // 최신 데이터 저장
        this.latestLocalPath = data;

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "로컬 경로 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/odometry")
    public ResponseEntity<?> receiveOdometry(@RequestBody Map<String, Object> data) {
        // 데이터 로깅 (필요시 주석 해제)
         log.info("오도메트리 데이터 수신: {}", data);

        // 최신 데이터 저장
        this.latestOdometry = data;

        try {
            // 오도메트리 구조에서 좌표 추출
            Map<String, Object> poseWrapper = (Map<String, Object>) data.get("pose");
            Map<String, Object> pose = (Map<String, Object>) poseWrapper.get("pose");
            Map<String, Object> position = (Map<String, Object>) pose.get("position");

            double x = ((Number) position.get("x")).doubleValue();
            double y = ((Number) position.get("y")).doubleValue();

            // ✅ 좌표 Redis 저장 + WebSocket 실시간 전송
            robotService.sendCurrentPosition("room1", x, y);

        } catch (Exception e) {
            log.error("오도메트리 좌표 파싱 실패", e);
        }

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "오도메트리 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/scan")
    public ResponseEntity<?> receiveScan(@RequestBody Map<String, Object> data) {
        // 데이터 로깅 (필요시 주석 해제)
        // log.info("스캔 데이터 수신: {}", data);

        // 최신 데이터 저장
        this.latestScan = data;

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "스캔 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/map")
    public ResponseEntity<?> receiveMap(@RequestBody Map<String, Object> data) {
        log.info("맵 데이터 수신: {}", data);

        // 1. 맵 info
        Map<String, Object> info = (Map<String, Object>) data.get("info");
        int width = (int) info.get("width");
        int height = (int) info.get("height");

        // 2. 전체 맵 -1로 초기화
        int[][] map = new int[height][width];
        for (int y = 0; y < height; y++) {
            Arrays.fill(map[y], -1);
        }

        // 3. 점유 셀 적용
        List<Map<String, Object>> occupiedCells = (List<Map<String, Object>>) data.get("occupied_cells");
        for (Map<String, Object> cell : occupiedCells) {
            int x = (int) cell.get("x");
            int y = (int) cell.get("y");
            int value = (int) cell.get("value");
            if (x >= 0 && x < width && y >= 0 && y < height) {
                map[y][x] = value;
            }
        }

        // 4. 전송용 객체 구성
        Map<String, Object> payload = new HashMap<>();
        payload.put("type", "map");
        payload.put("width", width);
        payload.put("height", height);
        payload.put("map", map);

        // 5. WebSocket 사용자들에게 실시간 전송
        userSocketHandler.broadcastMap(payload);

        return ResponseEntity.ok(Map.of("status", "success", "message", "맵 수신 및 전송 완료"));
    }


    @PostMapping("/auto-map")
    public ResponseEntity<?> startAutoMap(@RequestBody(required = false) Map<String, Object> command) {
        // 명령 데이터 준비 (command가 null인 경우 기본값 설정)
        boolean dataValue = true;
        if (command != null && command.containsKey("data")) {
            dataValue = Boolean.parseBoolean(command.get("data").toString());
        }

        log.info("자동 맵핑 명령: data={}", dataValue);

        // 브릿지 서버로 명령 전송
        Map<String, Object> requestData = new HashMap<>();
        requestData.put("command", "start_auto_map");
        requestData.put("data", dataValue);

        try {
            // 주입된 RestTemplate 사용 및 멤버 변수 bridgeUrl 사용
            ResponseEntity<Map> response = this.restTemplate.postForEntity(
                    this.bridgeUrl + "/command", requestData, Map.class);

            return ResponseEntity.ok(response.getBody());
        } catch (Exception e) {
            log.error("자동 맵핑 명령 전송 실패: {}", e.getMessage());
            Map<String, Object> error = new HashMap<>();
            error.put("status", "error");
            error.put("message", "자동 맵핑 명령을 전송하는 데 실패했습니다: " + e.getMessage());
            return ResponseEntity.status(500).body(error);
        }
    }

    @PostMapping("/stop-auto-map")
    public ResponseEntity<?> stopAutoMap(@RequestBody(required = false) Map<String, Object> command) {
        // 명령 데이터 준비 (command가 null인 경우 기본값 설정)
        boolean dataValue = true;
        if (command != null && command.containsKey("data")) {
            dataValue = Boolean.parseBoolean(command.get("data").toString());
        }

        log.info("자동 맵핑 정지 명령: data={}", dataValue);

        // 브릿지 서버로 명령 전송
        Map<String, Object> requestData = new HashMap<>();
        requestData.put("command", "stop_auto_map");
        requestData.put("data", dataValue);

        try {
            // 주입된 RestTemplate 사용 및 멤버 변수 bridgeUrl 사용
            ResponseEntity<Map> response = this.restTemplate.postForEntity(
                    this.bridgeUrl + "/command", requestData, Map.class);

            return ResponseEntity.ok(response.getBody());
        } catch (Exception e) {
            log.error("자동 맵핑 정지 명령 전송 실패: {}", e.getMessage());
            Map<String, Object> error = new HashMap<>();
            error.put("status", "error");
            error.put("message", "자동 맵핑 정지 명령을 전송하는 데 실패했습니다: " + e.getMessage());
            return ResponseEntity.status(500).body(error);
        }
    }

    @PostMapping("/mapping-done")
    public ResponseEntity<?> receiveMappingDone(@RequestBody Map<String, Object> data) {
        // 데이터 로깅
        log.info("맵핑 완료 데이터 수신: success={}", data.get("success"));

        // 최신 데이터 저장
        this.latestMappingDoneResult = data;

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "맵핑 완료 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/map-status")
    public ResponseEntity<?> receiveMapStatus(@RequestBody Map<String, Object> data) {
        // 데이터 로깅
        Double coverage = ((Number) data.get("coverage")).doubleValue();
        Double mapChangeRate = ((Number) data.get("map_change_rate")).doubleValue();
        Integer frontierCount = ((Number) data.get("frontier_count")).intValue();

        log.info("맵 상태 데이터 수신: coverage={}%, change_rate={}, frontiers={}",
                coverage, mapChangeRate, frontierCount);

        // 최신 데이터 저장
        this.latestMapStatus = data;

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "맵 상태 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/obstacle-alert")
    public ResponseEntity<?> receiveObstacleAlert(@RequestBody Map<String, Object> data) {
        // 데이터 로깅
        boolean detected = (boolean) data.get("detected");
        double distance = ((Number) data.get("distance")).doubleValue();

        if (detected) {
            log.info("장애물 감지 알림: 거리 {}m", distance);
        }

        // 최신 데이터 저장
        this.latestObstacleAlert = data;

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "장애물 감지 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/goal-status")
    public ResponseEntity<?> receiveGoalStatus(@RequestBody Map<String, Object> data) {
        // 데이터 로깅
        boolean reached = (boolean) data.get("reached");
        double timeTakenSec = ((Number) data.get("time_taken_sec")).doubleValue();

        if (reached) {
            log.info("목표 도달: 소요 시간 {}초", timeTakenSec);
        } else {
            log.info("목표 미달: 진행 시간 {}초", timeTakenSec);
        }

        // 최신 데이터 저장
        this.latestGoalStatus = data;

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "목표 도달 상태 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/pick-done")
    public ResponseEntity<?> receivePickDone(@RequestBody Map<String, Object> data) {
        // 데이터 로깅
        boolean success = (boolean) data.get("success");
        String productId = (String) data.get("product_id");
        String timestamp = (String) data.get("timestamp");

        if (success) {
            log.info("물건 집기 성공: 상품 ID {}, 시간 {}", productId, timestamp);
            storageService.autoReplenishFromStorage(Long.parseLong(productId));
        } else {
            log.info("물건 집기 실패: 상품 ID {}, 시간 {}", productId, timestamp);
        }

        // 최신 데이터 저장
        this.latestPickDone = data;

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "물건 집기 완료 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/place-done")
    public ResponseEntity<?> receivePlaceDone(@RequestBody Map<String, Object> data) {
        // 데이터 로깅
        boolean success = (boolean) data.get("success");
        int displaySpot = ((Number) data.get("display_spot")).intValue();
        String productId = (String) data.get("product_id");

        if (success) {
            log.info("전시 완료: 상품 ID {}, 진열 위치 {}", productId, displaySpot);
            // 여기서 자동 보충 트리거
            storageService.autoReplenishFromStorage(Long.parseLong(productId));
        } else {
            log.info("전시 실패: 상품 ID {}, 진열 위치 {}", productId, displaySpot);
        }

        // 최신 데이터 저장
        this.latestPlaceDone = data;

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "전시 완료 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }

    @PostMapping("/goal-pose")
    public ResponseEntity<?> setGoalPose(@RequestBody Map<String, Object> command) {
        try {
            // 명령 데이터 추출
            Map<String, Object> position = (Map<String, Object>) command.get("position");
            double x = ((Number) position.get("x")).doubleValue();
            double y = ((Number) position.get("y")).doubleValue();
            double orientation = ((Number) command.getOrDefault("orientation", 0.0)).doubleValue();

            log.info("목적지 설정 명령: x={}, y={}, orientation={}", x, y, orientation);

            // 브릿지 서버로 명령 전송
            Map<String, Object> requestData = new HashMap<>();
            requestData.put("command", "goal_pose");
            requestData.put("position", position);
            requestData.put("orientation", orientation);

            // 주입된 RestTemplate 사용 및 멤버 변수 bridgeUrl 사용
            ResponseEntity<Map> response = this.restTemplate.postForEntity(
                    this.bridgeUrl + "/command", requestData, Map.class);

            return ResponseEntity.ok(response.getBody());
        } catch (Exception e) {
            log.error("목적지 설정 명령 전송 실패: {}", e.getMessage());
            Map<String, Object> error = new HashMap<>();
            error.put("status", "error");
            error.put("message", "목적지 설정 명령을 전송하는 데 실패했습니다: " + e.getMessage());
            return ResponseEntity.status(500).body(error);
        }
    }


    // 시뮬브릿지 에서 백으로.
    @PostMapping("/image-jpeg-compressed")
    public ResponseEntity<?> receiveCompressedImage(@RequestBody Map<String, Object> data) {
        // 데이터 로깅 (필요시 주석 해제)
         log.info("압축된 JPEG 이미지 데이터 수신: {} bytes", ((String) data.get("data")).length());

        // 최신 데이터 저장
        this.latestCompressedImage = data;
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!여기는 브릿지에서 가져오기만함!!!db저장 구현할것

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "압축된 JPEG 이미지 데이터를 성공적으로 수신했습니다");
        return ResponseEntity.ok(response);
    }

    @PostMapping("/pick-place")
    public ResponseEntity<?> executePickPlaceCommand(@RequestBody Map<String, Object> command) {
        try {
            log.info("📦 전체 명령 데이터: {}", command);
            // 명령 데이터 추출
            Map<String, Object> from = (Map<String, Object>) command.get("from");
            Map<String, Object> to = (Map<String, Object>) command.get("to");
            String productId = (String) command.get("product_id");
            int displaySpot = ((Number) command.get("display_spot")).intValue();

            log.info("Pick and Place 명령: from={}, to={}, productId={}, displaySpot={}", from, to, productId, displaySpot);

            // 브릿지 서버로 명령 전송
            Map<String, Object> requestData = new HashMap<>();
            requestData.put("command", "pick_place");
            requestData.put("from", from);
            requestData.put("to", to);
            requestData.put("product_id", productId);
            requestData.put("display_spot", displaySpot);

            ResponseEntity<Map> response = this.restTemplate.postForEntity(
                    this.bridgeUrl + "/command", requestData, Map.class);
            return ResponseEntity.ok(response.getBody());
        } catch (Exception e) {
            log.error("Pick and Place 명령 전송 실패: {}", e.getMessage());
            Map<String, Object> error = new HashMap<>();
            error.put("status", "error");
            error.put("message", "Pick and Place 명령을 전송하는 데 실패했습니다: " + e.getMessage());
            return ResponseEntity.status(500).body(error);
        }
    }

    // ----- 데이터 조회용 GET 엔드포인트 -----

    @GetMapping("/global-path")
    public ResponseEntity<?> getGlobalPath() {
        return ResponseEntity.ok(this.latestGlobalPath);
    }

    @GetMapping("/local-path")
    public ResponseEntity<?> getLocalPath() {
        return ResponseEntity.ok(this.latestLocalPath);
    }

    @GetMapping("/odometry")
    public ResponseEntity<?> getOdometry() {
        return ResponseEntity.ok(this.latestOdometry);
    }

    @GetMapping("/scan")
    public ResponseEntity<?> getScan() {
        return ResponseEntity.ok(this.latestScan);
    }

    @GetMapping("/map")
    public ResponseEntity<?> getMap() {
        return ResponseEntity.ok(this.latestMap);
    }

    @GetMapping("/mapping-done")
    public ResponseEntity<?> getMappingDoneResult() {
        return ResponseEntity.ok(this.latestMappingDoneResult);
    }

    @GetMapping("/map-status")
    public ResponseEntity<?> getMapStatus() {
        return ResponseEntity.ok(this.latestMapStatus);
    }

    @GetMapping("/obstacle-alert")
    public ResponseEntity<?> getObstacleAlert() {
        return ResponseEntity.ok(this.latestObstacleAlert);
    }

    @GetMapping("/goal-status")
    public ResponseEntity<?> getGoalStatus() {
        return ResponseEntity.ok(this.latestGoalStatus);
    }

    @GetMapping("/pick-done")
    public ResponseEntity<?> getPickDone() {
        return ResponseEntity.ok(this.latestPickDone);
    }

    @GetMapping("/place-done")
    public ResponseEntity<?> getPlaceDone() {
        return ResponseEntity.ok(this.latestPlaceDone);
    }

    @GetMapping("/image-jpeg-compressed")
    //프론트에서 하고 싶은거... 실기간 연결??? 소켓사용해서 백 -> 프론 구현
    public ResponseEntity<?> getCompressedImage() {
        return ResponseEntity.ok(this.latestCompressedImage);
    }
}