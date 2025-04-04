package com.be.domain.robot.controller;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.client.RestTemplate;

import java.util.HashMap;
import java.util.Map;
import java.util.List;

@RestController
@RequestMapping("/api/robot")
public class RobotController {

    private final Logger log = LoggerFactory.getLogger(RobotController.class);
    private final RestTemplate restTemplate;

    // 브릿지 서버 URL 설정
    private final String bridgeUrl = "http://10.0.0.2:5000";

    // 최신 데이터 저장용 변수들
    private Map<String, Object> latestGlobalPath = new HashMap<>();
    private Map<String, Object> latestLocalPath = new HashMap<>();
    private Map<String, Object> latestOdometry = new HashMap<>();
    private Map<String, Object> latestScan = new HashMap<>();
    private Map<String, Object> latestMap = new HashMap<>();

    public RobotController(RestTemplate restTemplate) {
        this.restTemplate = restTemplate;
    }

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
        // log.info("오도메트리 데이터 수신: {}", data);

        // 최신 데이터 저장
        this.latestOdometry = data;

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
        // 데이터 로깅 (필요시 주석 해제)
        // log.info("맵 데이터 수신: {}", data);

        // 최신 데이터 저장
        this.latestMap = data;

        // 응답 생성
        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "맵 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
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
}