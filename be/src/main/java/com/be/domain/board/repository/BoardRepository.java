package com.be.domain.board.repository;

import com.be.domain.board.entity.Board;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Modifying;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;

import java.time.LocalDateTime;
import java.util.List;

@Repository
public interface BoardRepository extends JpaRepository<Board, Long> {
    List<Board> findByTitleContaining(String keyword);
    
    List<Board> findByContentContaining(String keyword);
    
    List<Board> findByTitleContainingOrContentContaining(String titleKeyword, String contentKeyword);
    
    List<Board> findAllByOrderByCreatedAtDesc();
    
    @Modifying
    @Query("UPDATE Board b SET b.viewCount = b.viewCount + 1 WHERE b.id = :id")
    void increaseViewCount(@Param("id") Long id);
    
    // 제목, 내용, 작성자로 검색
    Page<Board> findByTitleContainingOrContentContainingOrWriterContaining(
            String title, String content, String writer, Pageable pageable);
    
    // 만료되지 않은 공지사항만 검색
    Page<Board> findByExpirationDateIsNullOrExpirationDateGreaterThan(
            LocalDateTime now, Pageable pageable);
    
    // 만료되지 않은 공지사항 중에서 제목, 내용, 작성자로 검색
    Page<Board> findByExpirationDateIsNullOrExpirationDateGreaterThanAndTitleContainingOrContentContainingOrWriterContaining(
            LocalDateTime now, String title, String content, String writer, Pageable pageable);
} 
