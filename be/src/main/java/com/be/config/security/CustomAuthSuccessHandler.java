package com.be.config.security;

import com.be.domain.auth.dto.UserDto;
import com.be.domain.auth.utils.TokenUtils;
import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.servlet.http.Cookie;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.RequiredArgsConstructor;
import org.springframework.security.core.Authentication;
import org.springframework.security.web.authentication.AuthenticationSuccessHandler;
import org.springframework.stereotype.Component;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/*
사용자가 아이디/비밀번호로 로그인에 성공했을 때:

- Access Token을 생성해서 클라이언트에 응답
- Refresh Token을 HttpOnly 쿠키에 저장
- application/json 형태로 accessToken과 메시지를 응답
→ 클라이언트는 이걸 받고 상태 저장

*/


@Component
@RequiredArgsConstructor
public class CustomAuthSuccessHandler implements AuthenticationSuccessHandler {

    private final TokenUtils tokenUtils;

    @Override
    public void onAuthenticationSuccess(HttpServletRequest request,
                                        HttpServletResponse response,
                                        Authentication authentication) throws IOException {
        CustomUserDetails userDetails = (CustomUserDetails) authentication.getPrincipal();

        UserDto userDto = ((CustomUserDetails) authentication.getPrincipal()).toUserDto();

        // String accessToken = tokenUtils.generateAccessToken(
        //         userDto.getUserid(),
        //         userDto.getUsername()
        // );
        // //리프레시 토큰 생성
        // String refreshToken = tokenUtils.generateRefreshToken(userDto);
        // roles까지 포함해서 accessToken 생성
        String accessToken = tokenUtils.generateAccessToken(userDto);

        // refreshToken도 기존대로
        String refreshToken = tokenUtils.generateRefreshToken(userDto);

        // RefreshToken을 쿠키로 설정
        Cookie refreshCookie = new Cookie("refreshToken", refreshToken);
        refreshCookie.setHttpOnly(true);
        refreshCookie.setSecure(false); // 로컬 개발용. 배포 환경에선 true
        refreshCookie.setPath("/");
        refreshCookie.setMaxAge(7 * 24 * 60 * 60); // 7일

        response.addCookie(refreshCookie);

        // AccessToken은 응답 body로 전달
        response.setContentType("application/json");
        response.setCharacterEncoding("UTF-8");

        Map<String, String> tokenMap = new HashMap<>();
        tokenMap.put("accessToken", accessToken);
        tokenMap.put("username", userDto.getUsername());
        tokenMap.put("message", "로그인 성공");

        ObjectMapper objectMapper = new ObjectMapper();
        response.getWriter().write(objectMapper.writeValueAsString(tokenMap));
    }
}
