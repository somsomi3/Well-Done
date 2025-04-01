package com.be.domain.board.dto;

import lombok.Getter;
import lombok.Setter;
import java.time.LocalDateTime;

@Getter
@Setter
public class ExpirationDateRequestDto {
    private LocalDateTime expirationDate;
} 