package com.be.config.security;


import com.be.db.entity.User;
import com.be.domain.auth.dto.UserDto;
import lombok.Getter;
import org.springframework.security.core.GrantedAuthority;
import org.springframework.security.core.userdetails.UserDetails;

import java.util.Collection;
import java.util.Collections;

@Getter
public class CustomUserDetails implements UserDetails {

    private final User user;

    public CustomUserDetails(User user) {
        this.user = user;
    }

    @Override
    public Collection<? extends GrantedAuthority> getAuthorities() {
        return Collections.singleton(() -> "ROLE_USER"); // 권한 설정
    }

    @Override
    public String getPassword() {
        return user.getPassword();
    }

    @Override
    public String getUsername() {
        return user.getUsername();  // 혹은 이메일 등 식별 필드
    }


    // 계정 상태 관련 메서드 (기본 true로 설정)
    @Override
    public boolean isAccountNonExpired() { return true; }
    @Override
    public boolean isAccountNonLocked() { return true; }
    @Override
    public boolean isCredentialsNonExpired() { return true; }
    @Override
    public boolean isEnabled() { return true; }

    // User → UserDto 변환
    public UserDto toUserDto() {
        return UserDto.builder()
                .userid(user.getId())
                .username(user.getUsername())
                .email(user.getEmail()) // 필요 시 추가
                .build();
    }
    public Long getUserId() {
        return user.getId();
    }
}
