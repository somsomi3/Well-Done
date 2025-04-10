package com.be.domain.robot.service;

import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.stereotype.Service;

import java.util.Set;
import java.util.concurrent.TimeUnit;

@Slf4j
@Service
public class RedisService {

    private final RedisTemplate<String, Object> redisTemplate;

    // Redis 키 상수
    private static final String ROBOT_CAMERA_KEY = "robot:camera:latest";

    // 데이터 만료 시간 (분)
    private static final long CAMERA_EXPIRE_TIME = 5;

    // 레디스 맵 저장 키 상수
    private static final String MAPPING_DONE_KEY = "robot:mapping:done";
    private static final String MAP_KEY = "robot:map";
    private static final String MAP_INFLATED_KEY = "robot:map:inflated";

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

    /**
     * 맵핑 완료 데이터 저장
     * @param mappingDoneData JSON 형태의 맵핑 완료 데이터
     */
    public void saveMappingDoneData(String mappingDoneData) {
        try {
            redisTemplate.opsForValue().set(MAPPING_DONE_KEY, mappingDoneData);
        } catch (Exception e) {
            System.err.println("Redis에 맵핑 완료 데이터 저장 중 오류: " + e.getMessage());
        }
    }

    /**
     * 맵 데이터 저장
     * @param mapData JSON 형태의 맵 데이터
     * @param isInflated 인플레이트된 맵 여부
     */
    public void saveMapData(String mapData, boolean isInflated) {
        try {
            String key = isInflated ? MAP_INFLATED_KEY : MAP_KEY;
            redisTemplate.opsForValue().set(key, mapData);
        } catch (Exception e) {
            System.err.println("Redis에 맵 데이터 저장 중 오류: " + e.getMessage());
        }
    }

    /**
     * 맵핑 완료 데이터 조회
     * @return 맵핑 완료 데이터
     */
    public Object getMappingDoneData() {
        try {
            return redisTemplate.opsForValue().get(MAPPING_DONE_KEY);
        } catch (Exception e) {
            System.err.println("Redis에서 맵핑 완료 데이터 조회 중 오류: " + e.getMessage());
            return null;
        }
    }

    /**
     * 맵 데이터 조회
     * @param isInflated 인플레이트된 맵 여부
     * @return 맵 데이터
     */
    public Object getMapData(boolean isInflated) {
        try {
            String key = isInflated ? MAP_INFLATED_KEY : MAP_KEY;
            return redisTemplate.opsForValue().get(key);
        } catch (Exception e) {
            System.err.println("Redis에서 맵 데이터 조회 중 오류: " + e.getMessage());
            return null;
        }
    }
    /**
     * 맵핑 관련 데이터 삭제
     * 모든 맵핑 관련 데이터(원본, 일반 맵, 인플레이트된 맵)를 삭제합니다.
     */
    public void deleteMappingData() {
        try {
            // 등록된 키 상수 활용
            redisTemplate.delete(MAPPING_DONE_KEY);    // 원본 맵핑 데이터 키
            redisTemplate.delete(MAP_KEY);             // 일반 맵 데이터 키
            redisTemplate.delete(MAP_INFLATED_KEY);    // 인플레이트된 맵 데이터 키

            log.info("Redis에서 맵핑 관련 데이터 삭제 완료");
        } catch (Exception e) {
            log.error("맵핑 데이터 삭제 중 오류 발생", e);
            throw e;
        }
    }

    /**
     * 패턴으로 맵핑 관련 데이터 삭제
     * 지정된 패턴과 일치하는 모든 맵핑 관련 키를 삭제합니다.
     */
    public void deleteMappingDataByPattern(String pattern) {
        try {
            // 해당 패턴의 모든 키 조회
            Set<String> keys = redisTemplate.keys(pattern);

            if (keys != null && !keys.isEmpty()) {
                // 모든 관련 키 삭제
                redisTemplate.delete(keys);
                log.info("Redis에서 패턴 '{}' 일치하는 {} 개의 맵핑 데이터 삭제 완료", pattern, keys.size());
            } else {
                log.info("패턴 '{}'과 일치하는 삭제할 맵핑 데이터가 없습니다", pattern);
            }
        } catch (Exception e) {
            log.error("패턴 '{}'의 맵핑 데이터 삭제 중 오류 발생", pattern, e);
            throw e;
        }
    }

    /**
     * 모든 맵핑 관련 데이터 삭제
     * mapping:로 시작하는 모든 키를 삭제합니다.
     */
    public void deleteAllMappingData() {
        try {
            deleteMappingDataByPattern("robot:map*");
            deleteMappingDataByPattern(MAPPING_DONE_KEY);
        } catch (Exception e) {
            log.error("모든 맵핑 데이터 삭제 중 오류 발생", e);
            throw e;
        }
    }
}