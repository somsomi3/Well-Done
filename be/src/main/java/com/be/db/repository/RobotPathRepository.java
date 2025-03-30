package com.be.db.repository;

import java.util.List;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import com.be.db.entity.RobotPath;

@Repository
public interface RobotPathRepository extends JpaRepository<RobotPath, Long> {
	List<RobotPath> findByRobotIdOrderByTimestampAsc(Long robotId);
}
