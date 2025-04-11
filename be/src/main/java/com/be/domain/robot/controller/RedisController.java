package com.be.domain.robot.controller;

import com.be.domain.robot.service.RedisService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/api")
public class RedisController {

    @Autowired
    private RedisService redisService;

    @GetMapping("/saveToRedis")
    public String saveToRedis(@RequestParam String key, @RequestParam String value) {
        redisService.saveDataToRedis(key, value);
        return "Saved to Redis";
    }

    @GetMapping("/getFromRedis")
    public String getFromRedis(@RequestParam String key) {
        return (String) redisService.getDataFromRedis(key);
    }
}
