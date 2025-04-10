package com.be.domain.board.controller;

import com.be.domain.board.dto.BoardRequestDto;
import com.be.domain.board.dto.BoardResponseDto;
import com.be.domain.board.dto.ExpirationDateRequestDto;
import com.be.domain.board.service.BoardService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.PageRequest;
import org.springframework.data.domain.Pageable;
import org.springframework.data.domain.Sort;
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
    
    // 페이지네이션을 적용한 검색 엔드포인트
    @GetMapping("/search")
    public ResponseEntity<Page<BoardResponseDto>> searchBoardsWithPagination(
            @RequestParam(value = "keyword", required = false) String keyword,
            @RequestParam(value = "page", defaultValue = "0") int page,
            @RequestParam(value = "size", defaultValue = "10") int size,
            @RequestParam(value = "sort", defaultValue = "createdAt,desc") String sort) {
        
        String[] sortParams = sort.split(",");
        String sortBy = sortParams[0];
        String direction = sortParams.length > 1 ? sortParams[1] : "desc";
        
        Sort.Direction sortDirection = Sort.Direction.fromString(direction.toUpperCase());
        Pageable pageable = PageRequest.of(page, size, Sort.by(sortDirection, sortBy));
        
        Page<BoardResponseDto> responseDtos = boardService.searchBoardsWithPagination(keyword, pageable);
        return ResponseEntity.ok(responseDtos);
    }
    
    // 만료되지 않은 공지사항만 검색하는 엔드포인트
    @GetMapping("/active")
    public ResponseEntity<Page<BoardResponseDto>> searchActiveBoards(
            @RequestParam(value = "page", defaultValue = "0") int page,
            @RequestParam(value = "size", defaultValue = "10") int size,
            @RequestParam(value = "sort", defaultValue = "createdAt,desc") String sort) {
        
        String[] sortParams = sort.split(",");
        String sortBy = sortParams[0];
        String direction = sortParams.length > 1 ? sortParams[1] : "desc";
        
        Sort.Direction sortDirection = Sort.Direction.fromString(direction.toUpperCase());
        Pageable pageable = PageRequest.of(page, size, Sort.by(sortDirection, sortBy));
        
        Page<BoardResponseDto> responseDtos = boardService.searchActiveBoards(pageable);
        return ResponseEntity.ok(responseDtos);
    }
    
    // 만료되지 않은 공지사항 중에서 검색하는 엔드포인트
    @GetMapping("/active/search")
    public ResponseEntity<Page<BoardResponseDto>> searchActiveBoardsWithKeyword(
            @RequestParam(value = "keyword", required = false) String keyword,
            @RequestParam(value = "page", defaultValue = "0") int page,
            @RequestParam(value = "size", defaultValue = "10") int size,
            @RequestParam(value = "sort", defaultValue = "createdAt,desc") String sort) {
        
        String[] sortParams = sort.split(",");
        String sortBy = sortParams[0];
        String direction = sortParams.length > 1 ? sortParams[1] : "desc";
        
        Sort.Direction sortDirection = Sort.Direction.fromString(direction.toUpperCase());
        Pageable pageable = PageRequest.of(page, size, Sort.by(sortDirection, sortBy));
        
        Page<BoardResponseDto> responseDtos = boardService.searchActiveBoardsWithKeyword(keyword, pageable);
        return ResponseEntity.ok(responseDtos);
    }
    
    @PatchMapping("/{board_id}/expire")
    public ResponseEntity<BoardResponseDto> setExpirationDate(
            @PathVariable("board_id") Long id,
            @RequestBody ExpirationDateRequestDto requestDto) {
        System.out.println("요청 본문: " + requestDto);
        System.out.println("만료일: " + requestDto.getExpirationDate());
        
        try {
            BoardResponseDto responseDto = boardService.setExpirationDate(id, requestDto.getExpirationDate());
            System.out.println("응답 DTO: " + responseDto);
            System.out.println("응답 DTO의 만료일: " + responseDto.getExpirationDate());
            return ResponseEntity.ok(responseDto);
        } catch (Exception e) {
            System.err.println("만료일 설정 중 오류 발생: " + e.getMessage());
            e.printStackTrace();
            throw e;
        }
    }
}
