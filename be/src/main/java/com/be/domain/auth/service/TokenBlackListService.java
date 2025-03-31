package com.be.domain.auth.service;

import org.springframework.context.annotation.ComponentScan;
import org.springframework.stereotype.Service;

import java.util.List;

public interface TokenBlackListService {

    void addTokenToList(String value);

    boolean isContainToken(String value);

    List<Object> getTokenBlackList();

    void removeToken(String value);
}
