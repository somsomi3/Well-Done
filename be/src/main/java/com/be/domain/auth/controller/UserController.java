package com.be.domain.auth.controller;

import com.be.common.model.response.BaseResponseBody;
import com.be.config.CustomUserDetails;
import com.be.db.entity.User;
import com.be.db.repository.UserRepository;
import com.be.domain.auth.dto.UserDto;
import com.be.domain.auth.request.UpdateUserRequest;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.annotation.AuthenticationPrincipal;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/user")
@RequiredArgsConstructor
public class UserController {

    private final UserRepository userRepository;

    @GetMapping("/me")
    public ResponseEntity<BaseResponseBody<UserDto>> getUserInfo(Authentication authentication) {
        UserDto userDto = (UserDto) authentication.getPrincipal();
        return ResponseEntity.ok(BaseResponseBody.success(userDto, "회원 정보 조회 성공"));
    }



    @PutMapping("/update")
    public ResponseEntity<BaseResponseBody<UserDto>> updateUser(
            Authentication authentication,
            @RequestBody UpdateUserRequest request) {

        UserDto userDto = (UserDto) authentication.getPrincipal();

        User user = userRepository.findById(userDto.getUserid())
                .orElseThrow(() -> new RuntimeException("유저가 존재하지 않습니다"));

        user.setUsername(request.getUsername());
        user.setEmail(request.getEmail());
        userRepository.save(user);

        return ResponseEntity.ok(BaseResponseBody.success(new UserDto(user), "회원 정보 수정 완료"));

    }

}
