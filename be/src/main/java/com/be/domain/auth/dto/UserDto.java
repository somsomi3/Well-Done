package com.be.domain.auth.dto;

import com.be.db.entity.Role;
import com.be.db.entity.User;
import lombok.*;

import java.util.List;
import java.util.stream.Collectors;

@Getter
@Setter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class UserDto {
    private Long userid;           // 사용자 고유 ID
    private String username;       // 사용자 이름
    private String email;          // 이메일 주소
    private String password;       // 비밀번호 (프론트에 보내지 않을 수도 있음)
    private List<String> roles;    // 사용자 역할 리스트 (ex. ["ROLE_USER", "ROLE_ADMIN"])

    public UserDto(User user) {
        this.userid = user.getId();
        this.username = user.getUsername();
        this.email = user.getEmail();
        this.password = user.getPassword();

        // roles는 엔티티에서 문자열 이름으로 추출
        this.roles = user.getRoles().stream()
            .map(role -> role.getName().name()) // ROLE_USER, ROLE_ADMIN 등
            .collect(Collectors.toList());
    }
}
