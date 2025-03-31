package com.be.domain.robot.controller;

import java.util.List;

import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import com.be.db.entity.RobotPath;
import com.be.db.repository.RobotPathRepository;

import lombok.RequiredArgsConstructor;

@RestController
@RequestMapping("/robots")
@RequiredArgsConstructor
public class RobotController {

	private final RobotPathRepository robotPathRepository;

	@GetMapping("/{robotId}/map")
	public ResponseEntity<List<RobotPath>> getRobotPath(@PathVariable Long robotId) {
		List<RobotPath> pathList = robotPathRepository.findByRobotIdOrderByTimestampAsc(robotId);
		return ResponseEntity.ok(pathList);
	}
}
