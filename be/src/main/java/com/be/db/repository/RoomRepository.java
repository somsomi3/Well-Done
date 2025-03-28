package com.be.db.repository;

import java.util.Optional;

import com.be.db.entity.Room;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface RoomRepository extends JpaRepository<Room, Long> {

	// roomId를 기준으로 존재 여부 확인
	boolean existsByRoomId(String roomId);
	Optional<Room> findByRoomId(String roomId);

	// 필요 시: Room findByRoomId(String roomId); 도 추가 가능
}
