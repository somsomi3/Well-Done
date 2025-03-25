package com.be.domain.auth.utils;

import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
public class ValidTokenDto {
    private boolean isValid;     // 토큰 유효 여부
    private String errorName;    // 토큰 상태 (예: TOKEN_EXPIRED, TOKEN_INVALID, TOKEN_NULL)

    public static ValidTokenDto valid() {
        return new ValidTokenDto(true, null);
    }

    public static ValidTokenDto expired() {
        return new ValidTokenDto(false, "TOKEN_EXPIRED");
    }

    public static ValidTokenDto invalid() {
        return new ValidTokenDto(false, "TOKEN_INVALID");
    }

    public static ValidTokenDto nullToken() {
        return new ValidTokenDto(false, "TOKEN_NULL");
    }
}
