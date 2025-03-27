package com.be.config.security;

import com.be.domain.auth.dto.UserDto;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.extern.slf4j.Slf4j;
import org.springframework.security.authentication.AuthenticationManager;
import org.springframework.security.authentication.UsernamePasswordAuthenticationToken;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.AuthenticationException;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.security.web.authentication.UsernamePasswordAuthenticationFilter;
import org.springframework.stereotype.Component;

//아이디 비번으로 인증하는 필터(로그인 요청을 처리하는), HTTP, CORS

@Slf4j
@Component
public class CustomAuthenticationFilter extends UsernamePasswordAuthenticationFilter {

    public CustomAuthenticationFilter(AuthenticationManager authenticationManager) {
        super.setAuthenticationManager(authenticationManager);
        log.info("CustomAuthenticationFilter 초기화됨.");
    }

    @Override
    public Authentication attemptAuthentication(HttpServletRequest request, HttpServletResponse response) throws AuthenticationException {
        log.info("로그인 시도: {}", request.getRequestURI()); // 로그인 요청이 들어온 URI를 로그에 기록

        UsernamePasswordAuthenticationToken authRequest;
        try {
            authRequest = getAuthRequest(request);
            setDetails(request, authRequest);
        } catch (Exception e) {
            log.error("인증 요청 처리 중 오류 발생", e); // 예외 발생 시 로그에 기록
            throw new RuntimeException(e);
        }

        log.info("인증 요청 처리 중: username = {}", authRequest.getPrincipal()); // 인증 처리할 사용자 이름 출력
        return this.getAuthenticationManager().authenticate(authRequest);
    }


    private UsernamePasswordAuthenticationToken getAuthRequest(HttpServletRequest request) throws Exception {
        log.debug("getAuthRequest 시작"); // getAuthRequest 호출 시 로그
        try {
            ObjectMapper objectMapper = new ObjectMapper();
            objectMapper.configure(JsonParser.Feature.AUTO_CLOSE_SOURCE, true);
            UserDto user = objectMapper.readValue(request.getInputStream(), UserDto.class);

            log.debug("1.CustomAuthenticationFilter :: username: " + user.getUsername() + " userPw: " + user.getPassword());
            log.debug("사용자 정보 추출 :: username: {} | userPw: {}", user.getUsername(), user.getPassword());

            return new UsernamePasswordAuthenticationToken(user.getUsername(), user.getPassword());
        } catch (UsernameNotFoundException ae) {
            throw new UsernameNotFoundException(ae.getMessage());
        }
    }


}
