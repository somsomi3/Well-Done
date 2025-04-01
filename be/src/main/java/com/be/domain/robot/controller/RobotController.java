package com.be.domain.robot.controller;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.HashMap;
import java.util.Map;

@RestController
@RequestMapping("/api")
public class RobotController {

    private final Logger log = LoggerFactory.getLogger(RobotController.class);

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
        log.info("환경 상태 수신: month={}, day={}, hour={}, minute={}, temperature={}, weather={}",
                month, day, hour, minute, temperature, weather);

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
        log.info("터틀봇 상태 수신: {}", data);

        // 데이터 처리 로직
        // 예: 데이터베이스에 저장, 실시간 상태 업데이트 등

        Map<String, Object> response = new HashMap<>();
        response.put("status", "success");
        response.put("message", "터틀봇 상태 데이터를 성공적으로 수신했습니다");

        return ResponseEntity.ok(response);
    }
}
