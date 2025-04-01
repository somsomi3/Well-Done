package com.be.domain.board.controller;

import com.be.domain.board.dto.BoardRequestDto;
import com.be.domain.board.dto.BoardResponseDto;
import com.be.domain.board.dto.ExpirationDateRequestDto;
import com.be.domain.board.service.BoardService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.time.LocalDateTime;
import java.util.List;

@RestController //이 클래스가 웹 요청을 처리하고 반환된 객체를 자동으로 JSON으로 반환한다. @Controller와 @ResponseBody를 합친것과 같은 효과.
@RequestMapping("/api/boards/announcements")
public class BoardController {
    
    private final BoardService boardService;

    // 생성자를 통해서 의존성 주입
    @Autowired
    public BoardController(BoardService boardService) {
        this.boardService = boardService;
    }
    
    @PostMapping
    public ResponseEntity<BoardResponseDto> createBoard(@RequestBody BoardRequestDto requestDto) {
        BoardResponseDto responseDto = boardService.createBoard(requestDto);
        return ResponseEntity.status(HttpStatus.CREATED).body(responseDto);
    }
    
    @PatchMapping("/{board_id}")
    public ResponseEntity<BoardResponseDto> updateBoard(
            @PathVariable("board_id") Long id,
            @RequestBody BoardRequestDto requestDto) {
        BoardResponseDto responseDto = boardService.updateBoard(id, requestDto);
        return ResponseEntity.ok(responseDto);
    }
    
    @DeleteMapping("/{board_id}")
    public ResponseEntity<Void> deleteBoard(@PathVariable("board_id") Long id) {
        boardService.deleteBoard(id);
        return ResponseEntity.noContent().build();
    }
    
    @GetMapping("/{board_id}")
    public ResponseEntity<BoardResponseDto> getBoard(@PathVariable("board_id") Long id) {
        boardService.increaseViewCount(id);
        BoardResponseDto responseDto = boardService.getBoard(id);
        return ResponseEntity.ok(responseDto);
    }
    
    @GetMapping
    public ResponseEntity<List<BoardResponseDto>> searchBoards(
            @RequestParam(value = "query", required = false) String keyword) {
        List<BoardResponseDto> responseDtos = boardService.searchBoards(keyword);
        return ResponseEntity.ok(responseDtos);
    }
    
    @PatchMapping("/{board_id}/expire")
    public ResponseEntity<BoardResponseDto> setExpirationDate(
            @PathVariable("board_id") Long id,
            @RequestBody ExpirationDateRequestDto requestDto) {
        BoardResponseDto responseDto = boardService.setExpirationDate(id, requestDto.getExpirationDate());
        return ResponseEntity.ok(responseDto);
    }
} 