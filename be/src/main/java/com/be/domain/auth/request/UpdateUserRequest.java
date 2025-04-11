package com.be.domain.auth.request;

import lombok.Data;

@Data
public class UpdateUserRequest {
    private String username;
    private String email;
}
