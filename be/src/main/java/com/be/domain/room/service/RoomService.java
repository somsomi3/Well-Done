package com.be.domain.room.service;

import org.springframework.stereotype.Service;
import com.be.db.entity.Room;
import com.be.db.repository.RoomRepository;

import jakarta.transaction.Transactional;
import lombok.RequiredArgsConstructor;

@Transactional
@Service
@RequiredArgsConstructor
public class RoomService {

	private final RoomRepository roomRepository;

	// 자동으로 사용 가능한 방 번호를 찾아서 생성
	public String createNextAvailableRoom(String username) {
		try {
			int robotNumber = 0;

			// 사용 중인 번호 피해서 비어있는 번호 찾기
			while (roomRepository.existsByRoomId(generateRoomId(username, robotNumber))) {
				robotNumber++;
			}

			// 찾은 번호로 방 생성
			String roomId = generateRoomId(username, robotNumber);

			Room room = Room.builder()
				.roomId(roomId)
				.userId(username)
				.robotNumber(robotNumber)
				.build();

			roomRepository.save(room);
			System.out.println("자동 방 생성 완료: " + roomId);
			return roomId;

		} catch (Exception e) {
			System.out.println("RoomService 예외 발생: " + e.getMessage());
			throw e;
		}
	}

	// 고정된 방식도 남겨두기 (필요시 직접 지정 가능)
	public String createRoom(String username, int robotNumber) {
		String roomId = generateRoomId(username, robotNumber);
		if (roomRepository.existsByRoomId(roomId)) {
			throw new IllegalArgumentException("이미 존재하는 방입니다.");
		}

		Room room = Room.builder()
			.roomId(roomId)
			.userId(username)
			.robotNumber(robotNumber)
			.build();

		roomRepository.save(room);
		System.out.println("방이 생성되어 저장되었습니다: " + roomId);
		return roomId;
	}

	public String generateRoomId(String username, int robotNumber) {
		return username + "_robot" + robotNumber;
	}
}
