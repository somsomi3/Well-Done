package com.be.domain.board.dto;

import lombok.Getter;
import lombok.Setter;
import java.time.LocalDateTime;

@Getter
@Setter
public class BoardRequestDto {
    private String title;
    private String content;
    private String writer;
    private LocalDateTime expirationDate;
}
