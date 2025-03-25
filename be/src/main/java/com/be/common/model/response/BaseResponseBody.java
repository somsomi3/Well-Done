package com.be.common.model.response;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
public class BaseResponseBody<T> {
    private int statusCode;  // HTTP 상태 코드 (ex: 200, 400, 500)
    private String message;  // 응답 메시지 (ex: "성공", "잘못된 요청")
    private T data;          // 응답 데이터 (제네릭 타입 사용)

    // 성공 응답을 위한 정적 메서드
    public static <T> BaseResponseBody<T> success(T data, String message) {
        return new BaseResponseBody<>(200, message, data); // 메시지 반영
    }

    // 실패 응답을 위한 정적 메서드 (커스텀 메시지 포함)
    public static <T> BaseResponseBody<T> error(int statusCode, String message) {
        return new BaseResponseBody<>(statusCode, message, null);
    }
}
