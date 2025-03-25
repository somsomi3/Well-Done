package com.be.domain.robot.service;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.stereotype.Service;

@Service
public class RedisService {

    private final RedisTemplate<String, Object> redisTemplate;

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


}