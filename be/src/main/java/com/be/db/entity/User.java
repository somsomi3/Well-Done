package com.be.db.entity;

import jakarta.persistence.*;
import lombok.*;
import org.springframework.security.crypto.bcrypt.BCryptPasswordEncoder;

import java.util.List;

@Entity
@Builder
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
public class User extends BaseEntity {

    @Column(nullable = false, unique = true)
    private String userId;
    // private String userId = UUID.randomUUID().toString();  // UUID 생성

    @Column(nullable = false, unique = true)
    private String email; // 이메일은 반드시 UNIQUE

    @Column(nullable = false)
    private String password; // NOT NULL 적용

    @Column(nullable = false, length = 100)
    private String username; // NOT NULL 추가

    @Column(nullable = false)
    private String companyId;  // 회사 코드 혹은 사업자번호
    // 연관관계 설정

    @ManyToMany(fetch = FetchType.EAGER)
    // @ManyToOne(fetch = FetchType.EAGER)
    @JoinTable(
        name = "user_roles",
        joinColumns = @JoinColumn(name = "user_id"),
        inverseJoinColumns = @JoinColumn(name = "role_id")
    )
    private List<Role> roles;
}