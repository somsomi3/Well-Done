package com.be.domain.board.service;

import com.be.domain.board.dto.BoardRequestDto;
import com.be.domain.board.dto.BoardResponseDto;
import com.be.domain.board.entity.Board;
import com.be.domain.board.repository.BoardRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;
import java.util.stream.Collectors;

@Service
public class BoardService {
    
    private final BoardRepository boardRepository;
    
    @Autowired
    public BoardService(BoardRepository boardRepository) {
        this.boardRepository = boardRepository;
    }
    
    @Transactional
    public BoardResponseDto createBoard(BoardRequestDto requestDto) {
        Board board = new Board();
        board.setTitle(requestDto.getTitle());
        board.setContent(requestDto.getContent());
        board.setWriter(requestDto.getWriter());
        board.setExpirationDate(requestDto.getExpirationDate());
        
        Board savedBoard = boardRepository.save(board);
        return BoardResponseDto.fromEntity(savedBoard);
    }
    
    @Transactional
    public BoardResponseDto updateBoard(Long id, BoardRequestDto requestDto) {
        Board board = boardRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("공지사항을 찾을 수 없습니다."));
        
        board.setTitle(requestDto.getTitle());
        board.setContent(requestDto.getContent());
        board.setWriter(requestDto.getWriter());
        board.setExpirationDate(requestDto.getExpirationDate());
        
        Board updatedBoard = boardRepository.save(board);
        return BoardResponseDto.fromEntity(updatedBoard);
    }
    
    @Transactional
    public void deleteBoard(Long id) {
        Board board = boardRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("공지사항을 찾을 수 없습니다."));
        
        boardRepository.delete(board);
    }
    
    @Transactional
    public void increaseViewCount(Long id) {
        boardRepository.increaseViewCount(id);
    }
    
    @Transactional(readOnly = true)
    public BoardResponseDto getBoard(Long id) {
        Board board = boardRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("공지사항을 찾을 수 없습니다."));
        
        return BoardResponseDto.fromEntity(board);
    }

    @Transactional(readOnly = true)
    public List<BoardResponseDto> searchBoards(String keyword) {
        List<Board> boards;

        if (keyword == null || keyword.isEmpty()) {
            // 키워드가 제공되지 않은 경우, 모든 게시판 반환
            boards = boardRepository.findAllByOrderByCreatedAtDesc();
        } else {
            // 키워드가 제공된 경우, 제목 또는 내용으로 검색
            // 제목으로 검색
            List<Board> titleResults = boardRepository.findByTitleContaining(keyword);
            // 내용으로 검색
            List<Board> contentResults = boardRepository.findByContentContaining(keyword);
            
            // 두 결과를 합치고 중복 제거
            boards = titleResults;
            for (Board board : contentResults) {
                if (!titleResults.contains(board)) {
                    boards.add(board);
                }
            }
        }

        return boards.stream()
            .map(BoardResponseDto::fromEntity)
            .collect(Collectors.toList());
    }
    
    // 페이지네이션을 적용한 검색 메서드
    @Transactional(readOnly = true)
    public Page<BoardResponseDto> searchBoardsWithPagination(String keyword, Pageable pageable) {
        Page<Board> boards;
        
        if (keyword == null || keyword.isEmpty()) {
            // 키워드가 제공되지 않은 경우, 모든 게시판 반환
            boards = boardRepository.findAll(pageable);
        } else {
            // 키워드가 제공된 경우, 제목, 내용, 작성자로 검색
            boards = boardRepository.findByTitleContainingOrContentContainingOrWriterContaining(
                    keyword, keyword, keyword, pageable);
        }
        
        return boards.map(BoardResponseDto::fromEntity);
    }
    
    // 만료되지 않은 공지사항만 검색
    @Transactional(readOnly = true)
    public Page<BoardResponseDto> searchActiveBoards(Pageable pageable) {
        Page<Board> boards = boardRepository.findByExpirationDateIsNullOrExpirationDateGreaterThan(
                LocalDateTime.now(), pageable);
        
        return boards.map(BoardResponseDto::fromEntity);
    }
    
    // 만료되지 않은 공지사항 중에서 검색
    @Transactional(readOnly = true)
    public Page<BoardResponseDto> searchActiveBoardsWithKeyword(String keyword, Pageable pageable) {
        Page<Board> boards;
        
        if (keyword == null || keyword.isEmpty()) {
            // 키워드가 제공되지 않은 경우, 만료되지 않은 모든 공지사항 반환
            boards = boardRepository.findByExpirationDateIsNullOrExpirationDateGreaterThan(
                    LocalDateTime.now(), pageable);
        } else {
            // 키워드가 제공된 경우, 만료되지 않은 공지사항 중에서 제목, 내용, 작성자로 검색
            boards = boardRepository.findByExpirationDateIsNullOrExpirationDateGreaterThanAndTitleContainingOrContentContainingOrWriterContaining(
                    LocalDateTime.now(), keyword, keyword, keyword, pageable);
        }
        
        return boards.map(BoardResponseDto::fromEntity);
    }
    
    @Transactional
    public BoardResponseDto setExpirationDate(Long id, LocalDateTime expirationDate) {
        Board board = boardRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("공지사항을 찾을 수 없습니다."));
        
        System.out.println("설정 전 만료일: " + board.getExpirationDate());
        System.out.println("설정할 만료일: " + expirationDate);
        
        board.setExpirationDate(expirationDate);
        Board updatedBoard = boardRepository.save(board);
        
        System.out.println("설정 후 만료일: " + updatedBoard.getExpirationDate());
        
        return BoardResponseDto.fromEntity(updatedBoard);
    }
} 
