package com.be.domain.board.dto;

import com.be.domain.board.entity.Board;
import lombok.Getter;
import lombok.Setter;
import java.time.LocalDateTime;

@Getter
@Setter
public class BoardResponseDto {
    private Long id;
    private String title;
    private String content;
    private String writer;
    private Integer viewCount;
    private LocalDateTime createdAt;
    private LocalDateTime updatedAt;
    private LocalDateTime expirationDate;
    
    public static BoardResponseDto fromEntity(Board board) {
        BoardResponseDto dto = new BoardResponseDto();
        dto.setId(board.getId());
        dto.setTitle(board.getTitle());
        dto.setContent(board.getContent());
        dto.setWriter(board.getWriter());
        dto.setViewCount(board.getViewCount());
        dto.setCreatedAt(board.getCreatedAt());
        dto.setUpdatedAt(board.getUpdatedAt());
        dto.setExpirationDate(board.getExpirationDate());
        
        System.out.println("Board 엔티티의 만료일: " + board.getExpirationDate());
        System.out.println("DTO로 변환된 만료일: " + dto.getExpirationDate());
        
        return dto;
    }
} 
