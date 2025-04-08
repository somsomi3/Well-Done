package com.be.domain.robot.service;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.stereotype.Service;

import java.util.concurrent.TimeUnit;

@Service
public class RedisService {

    private final RedisTemplate<String, Object> redisTemplate;

    // Redis 키 상수
    private static final String ROBOT_CAMERA_KEY = "robot:camera:latest";

    // 데이터 만료 시간 (분)
    private static final long CAMERA_EXPIRE_TIME = 5;

    @Autowired
    public RedisService(RedisTemplate<String, Object> redisTemplate) {
        this.redisTemplate = redisTemplate;
    }

    // 로봇의 실시간 위치를 Redis에 저장
    public void saveRobotPosition(String roomId, Object position) {
        redisTemplate.opsForHash().put(roomId, "robot_position", position);
    }

    // Redis에서 로봇의 실시간 위치를 조회
    public Object getRobotPosition(String roomId) {
        return redisTemplate.opsForHash().get(roomId, "robot_position");
    }

    // Redis에서 로봇의 경로 전체 지도 가져오기
    public Object getRobotPath(String roomId) {
        return redisTemplate.opsForHash().get(roomId, "robot_path");
    }

    // Redis에서 데이터를 조회
    public Object getDataFromRedis(String key) {
        return redisTemplate.opsForValue().get(key);
    }
    // 일반적인 데이터를 Redis에 저장
    public void saveDataToRedis(String key, Object value) {
        redisTemplate.opsForValue().set(key, value);
    }

    /**
     * 로봇 카메라 이미지 데이터 저장
     * @param imageData 이미지 데이터
     */
    public void saveRobotCameraImage(Object imageData) {
        try {
            redisTemplate.opsForValue().set(ROBOT_CAMERA_KEY, imageData);
            redisTemplate.expire(ROBOT_CAMERA_KEY, CAMERA_EXPIRE_TIME, TimeUnit.MINUTES);
        } catch (Exception e) {
            // 로깅 또는 예외 처리
            System.err.println("Redis에 카메라 이미지 저장 중 오류: " + e.getMessage());
        }
    }

    /**
     * 로봇 카메라 이미지 데이터 조회
     * @return 이미지 데이터
     */
    public Object getRobotCameraImage() {
        try {
            return redisTemplate.opsForValue().get(ROBOT_CAMERA_KEY);
        } catch (Exception e) {
            // 로깅 또는 예외 처리
            System.err.println("Redis에서 카메라 이미지 조회 중 오류: " + e.getMessage());
            return null;
        }
    }

}