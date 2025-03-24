package com.be.db.repository;

import com.be.db.entity.User;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.Optional;

@Repository
public interface UserRepository extends JpaRepository<User, Long> {

    // 유저네임을 기준으로 사용자 조회 (로그인 시 사용)
    Optional<User> findByUsername(String username);

    // 이메일 중복 확인 (회원가입 시 사용)
    boolean existsByEmail(String email);
}
