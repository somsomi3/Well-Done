package com.be.domain.room.controller;

import org.springframework.http.ResponseEntity;
import org.springframework.security.core.annotation.AuthenticationPrincipal;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import com.be.domain.auth.dto.UserDto;
import com.be.domain.room.service.RoomService;

import jakarta.transaction.Transactional;
import lombok.RequiredArgsConstructor;

@RestController
@RequestMapping("/rooms")
@RequiredArgsConstructor
public class RoomController {

	private final RoomService roomService;

	@PostMapping("/create")
	@Transactional
	public ResponseEntity<String> createRoom(@AuthenticationPrincipal UserDto userDto) {
		String username = userDto.getUsername();
		System.out.println("!!!컨트롤러 요청 - username: " + username);

		// robotNumber 자동 할당 버전
		String roomId = roomService.createNextAvailableRoom(username);

		return ResponseEntity.ok(roomId);
	}
}
