package com.be.domain.board.service;

import com.be.domain.board.dto.BoardRequestDto;
import com.be.domain.board.dto.BoardResponseDto;
import com.be.domain.board.entity.Board;
import com.be.domain.board.repository.BoardRepository;
import org.springframework.beans.factory.annotation.Autowired;
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
        List<Board> boards = boardRepository.findByTitleContaining(keyword);
        
        return boards.stream()
                .map(BoardResponseDto::fromEntity)
                .collect(Collectors.toList());
    }
    
    @Transactional
    public BoardResponseDto setExpirationDate(Long id, LocalDateTime expirationDate) {
        Board board = boardRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("공지사항을 찾을 수 없습니다."));
        
        board.setExpirationDate(expirationDate);
        Board updatedBoard = boardRepository.save(board);
        return BoardResponseDto.fromEntity(updatedBoard);
    }
} 