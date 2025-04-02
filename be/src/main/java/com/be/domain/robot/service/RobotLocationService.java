package com.be.domain.robot.service;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.stereotype.Service;

import com.be.db.entity.RobotPath;
import com.be.db.repository.RobotPathRepository;
import com.be.domain.robot.UserSocketHandler;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

@Service
@RequiredArgsConstructor
@Slf4j
public class RobotLocationService {

	private final RedisTemplate<String, String> redisTemplate;
	private final RobotPathRepository robotPathRepository;
	private final UserSocketHandler userSocketHandler;
	private final ObjectMapper objectMapper = new ObjectMapper();

	private final Map<Long, Long> lastSaveTime = new ConcurrentHashMap<>();

	public void handleLocation(Long robotId, JsonNode location) {
		try {
			// 좌표 파싱
			double x = location.get("x").asDouble();
			double y = location.get("y").asDouble();

			// 1. Redis에 실시간 저장
			String redisKey = "robot:" + robotId + ":path";
			redisTemplate.opsForList().rightPush(redisKey, location.toString());

			// 2. 주기적 DB 저장 (5초 간격)
			long now = System.currentTimeMillis();
			long lastTime = lastSaveTime.getOrDefault(robotId, 0L);

			if (now - lastTime >= 5000) {
				lastSaveTime.put(robotId, now);
				RobotPath path = new RobotPath(robotId, x, y);
				robotPathRepository.save(path);
				log.info("DB에 위치 저장: {}", path);
			}

			// 3. WebSocket으로 프론트에 실시간 전송
			ObjectNode msg = objectMapper.createObjectNode();
			msg.put("type", "robot_location");
			msg.put("robotId", robotId);
			msg.set("position", location);

			// userSocketHandler.broadcastAll(new TextMessage(msg.toString()));
		} catch (Exception e) {
			log.error("로봇 위치 처리 실패", e);
		}
	}
}

