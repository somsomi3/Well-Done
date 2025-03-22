package com.be.domain.auth.dto;

import lombok.*;

@Getter
@Setter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class UserDto {
    private Long userid;           // 사용자 고유 ID
    private String username;       // 사용자 이름
    private String email;          // 이메일 주소
    private String password;       // 비밀번호 (암호화된 형태로 저장되어야 함)
    private String role;           // 사용자 역할 (예: ADMIN, USER)

    // 필요한 경우에만 추가 필드들 (예: 사용자 상태, 전화번호 등)
}
