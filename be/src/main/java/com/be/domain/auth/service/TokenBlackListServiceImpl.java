package com.be.domain.auth.service;

import org.springframework.stereotype.Service;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@Service  // ✅ 여기 붙여야 함
public class TokenBlackListServiceImpl implements TokenBlackListService {

    private final Set<String> blackList = new HashSet<>();

    @Override
    public void addTokenToList(String value) {
        blackList.add(value);
    }

    @Override
    public boolean isContainToken(String value) {
        return blackList.contains(value);
    }

    @Override
    public List<Object> getTokenBlackList() {
        return new ArrayList<>(blackList);
    }

    @Override
    public void removeToken(String value) {
        blackList.remove(value);
    }
}
