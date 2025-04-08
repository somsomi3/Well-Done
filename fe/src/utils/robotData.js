// utils/robotData.js
export const robotData = {
    robots: [
      {
        id: 1,
        activities: [
          {
            task_id: "TASK-0501-1",
            type: "empty_pallet",
            timestamp: "2024-05-28T09:15:00",
            start_location: "A-3",
            end_location: "Pallet Storage-2",
            status: "completed",
            duration_minutes: 18,
            items_moved: 24,
            notes: "포카칩 30g 24박스 이동"
          },
          {
            task_id: "TASK-0501-2",
            type: "restock",
            timestamp: "2024-05-28T11:30:00",
            start_location: "Warehouse-C",
            end_location: "B-2",
            status: "completed",
            duration_minutes: 32,
            items_moved: 48,
            notes: "오레오 쿠키 12팩 48개 보충"
          },
          {
            task_id: "TASK-0501-3",
            type: "combined",
            timestamp: "2024-05-28T14:20:00",
            start_location: "C-1",
            end_location: "Pallet Storage-1",
            status: "in_progress",
            duration_minutes: 45,
            items_moved: 36,
            notes: "스윙칩 + 허니버터칩 복합 이동"
          },
          {
            task_id: "TASK-0502-1",
            type: "empty_pallet",
            timestamp: "2024-05-29T10:00:00",
            start_location: "D-4",
            end_location: "Pallet Storage-3",
            status: "completed",
            duration_minutes: 22,
            items_moved: 18,
            notes: "콘칩 빈 파렛트 3개 처리"
          },
          {
            task_id: "TASK-0502-2",
            type: "restock",
            timestamp: "2024-05-29T13:45:00",
            start_location: "Warehouse-A",
            end_location: "E-5",
            status: "completed",
            duration_minutes: 28,
            items_moved: 60,
            notes: "홈런볼 60개 즉시 보충"
          },
          {
            task_id: "TASK-0503-1",
            type: "empty_pallet",
            timestamp: "2024-05-30T08:30:00",
            start_location: "F-2",
            end_location: "Pallet Storage-4",
            status: "pending",
            duration_minutes: 0,
            items_moved: 0,
            notes: "새우깡 파렛트 이동 대기"
          },
          {
            task_id: "TASK-0503-2",
            type: "restock",
            timestamp: "2024-05-30T10:15:00",
            start_location: "Warehouse-B",
            end_location: "G-7",
            status: "in_progress",
            duration_minutes: 15,
            items_moved: 36,
            notes: "꼬북칩 36팩 적재 중"
          },
          {
            task_id: "TASK-0503-3",
            type: "combined",
            timestamp: "2024-05-30T14:00:00",
            start_location: "H-1",
            end_location: "Pallet Storage-2",
            status: "completed",
            duration_minutes: 40,
            items_moved: 42,
            notes: "양파링/감자칩 동시 이동"
          },
          {
            task_id: "TASK-0504-1",
            type: "empty_pallet",
            timestamp: "2024-05-31T09:00:00",
            start_location: "I-3",
            end_location: "Pallet Storage-5",
            status: "completed",
            duration_minutes: 25,
            items_moved: 30,
            notes: "빈 파렛트 5개 회수 완료"
          },
          {
            task_id: "TASK-0504-2",
            type: "restock",
            timestamp: "2024-05-31T11:20:00",
            start_location: "Warehouse-D",
            end_location: "J-4",
            status: "pending",
            duration_minutes: 0,
            items_moved: 0,
            notes: "초코파이 120개 예정"
          },
          {
            task_id: "TASK-0504-3",
            type: "combined",
            timestamp: "2024-05-31T15:30:00",
            start_location: "K-2",
            end_location: "Pallet Storage-1",
            status: "in_progress",
            duration_minutes: 20,
            items_moved: 24,
            notes: "오징어칩 + 바나나킥 복합 작업"
          },
          {
            task_id: "TASK-0505-1",
            type: "empty_pallet",
            timestamp: "2024-06-01T08:45:00",
            start_location: "L-5",
            end_location: "Pallet Storage-3",
            status: "completed",
            duration_minutes: 18,
            items_moved: 18,
            notes: "빈 파렛트 3개 이동 완료"
          },
          {
            task_id: "TASK-0505-2",
            type: "restock",
            timestamp: "2024-06-01T12:00:00",
            start_location: "Warehouse-E",
            end_location: "M-6",
            status: "completed",
            duration_minutes: 35,
            items_moved: 72,
            notes: "초코에몽 72팩 보충"
          },
          {
            task_id: "TASK-0505-3",
            type: "combined",
            timestamp: "2024-06-01T16:15:00",
            start_location: "N-1",
            end_location: "Pallet Storage-4",
            status: "pending",
            duration_minutes: 0,
            items_moved: 0,
            notes: "맛동산/썬칩 복합 작업 예정"
          },
          {
            task_id: "TASK-0506-1",
            type: "empty_pallet",
            timestamp: "2024-06-02T10:30:00",
            start_location: "O-2",
            end_location: "Pallet Storage-2",
            status: "in_progress",
            duration_minutes: 12,
            items_moved: 12,
            notes: "빈 파렛트 2개 이동 중"
          }
        ]
      },
      {
        id: 2,
        activities: [
          {
            task_id: "TASK-0501-4",
            type: "empty_pallet",
            timestamp: "2024-05-28T10:45:00",
            start_location: "P-3",
            end_location: "Pallet Storage-6",
            status: "completed",
            duration_minutes: 20,
            items_moved: 15,
            notes: "아몬드 빈 파렛트 처리"
          },
          {
            task_id: "TASK-0501-5",
            type: "restock",
            timestamp: "2024-05-28T13:00:00",
            start_location: "Warehouse-F",
            end_location: "Q-4",
            status: "completed",
            duration_minutes: 25,
            items_moved: 40,
            notes: "카스타드 40개 보충"
          },
          {
            task_id: "TASK-0501-6",
            type: "combined",
            timestamp: "2024-05-28T16:30:00",
            start_location: "R-1",
            end_location: "Pallet Storage-5",
            status: "in_progress",
            duration_minutes: 30,
            items_moved: 18,
            notes: "땅콩/호두 복합 이동"
          },
          {
            task_id: "TASK-0502-3",
            type: "empty_pallet",
            timestamp: "2024-05-29T11:15:00",
            start_location: "S-2",
            end_location: "Pallet Storage-7",
            status: "completed",
            duration_minutes: 22,
            items_moved: 24,
            notes: "견과류 빈 파렛트 처리"
          },
          {
            task_id: "TASK-0502-4",
            type: "restock",
            timestamp: "2024-05-29T14:30:00",
            start_location: "Warehouse-G",
            end_location: "T-5",
            status: "completed",
            duration_minutes: 28,
            items_moved: 50,
            notes: "다크초콜릿 50개 적재"
          },
          {
            task_id: "TASK-0503-4",
            type: "empty_pallet",
            timestamp: "2024-05-30T09:45:00",
            start_location: "U-3",
            end_location: "Pallet Storage-8",
            status: "pending",
            duration_minutes: 0,
            items_moved: 0,
            notes: "건포도 파렛트 이동 예정"
          },
          {
            task_id: "TASK-0503-5",
            type: "restock",
            timestamp: "2024-05-30T12:00:00",
            start_location: "Warehouse-H",
            end_location: "V-6",
            status: "in_progress",
            duration_minutes: 18,
            items_moved: 24,
            notes: "화이트초콜릿 24개 적재 중"
          },
          {
            task_id: "TASK-0503-6",
            type: "combined",
            timestamp: "2024-05-30T15:15:00",
            start_location: "W-1",
            end_location: "Pallet Storage-6",
            status: "completed",
            duration_minutes: 35,
            items_moved: 30,
            notes: "아몬드/캐슈넛 복합 작업"
          },
          {
            task_id: "TASK-0504-4",
            type: "empty_pallet",
            timestamp: "2024-05-31T10:30:00",
            start_location: "X-4",
            end_location: "Pallet Storage-9",
            status: "completed",
            duration_minutes: 20,
            items_moved: 18,
            notes: "빈 파렛트 3개 회수"
          },
          {
            task_id: "TASK-0504-5",
            type: "restock",
            timestamp: "2024-05-31T13:45:00",
            start_location: "Warehouse-I",
            end_location: "Y-7",
            status: "pending",
            duration_minutes: 0,
            items_moved: 0,
            notes: "마카다미아 60개 예정"
          },
          {
            task_id: "TASK-0504-6",
            type: "combined",
            timestamp: "2024-05-31T16:00:00",
            start_location: "Z-2",
            end_location: "Pallet Storage-7",
            status: "in_progress",
            duration_minutes: 25,
            items_moved: 21,
            notes: "피칸/잣 복합 이동 작업"
          },
          {
            task_id: "TASK-0505-4",
            type: "empty_pallet",
            timestamp: "2024-06-01T09:15:00",
            start_location: "AA-5",
            end_location: "Pallet Storage-10",
            status: "completed",
            duration_minutes: 18,
            items_moved: 12,
            notes: "빈 파렛트 2개 처리 완료"
          },
          {
            task_id: "TASK-0505-5",
            type: "restock",
            timestamp: "2024-06-01T12:30:00",
            start_location: "Warehouse-J",
            end_location: "BB-6",
            status: "completed",
            duration_minutes: 30,
            items_moved: 45,
            notes: "견과류 믹스 45개 보충"
          },
          {
            task_id: "TASK-0505-6",
            type: "combined",
            timestamp: "2024-06-01T15:45:00",
            start_location: "CC-1",
            end_location: "Pallet Storage-8",
            status: "pending",
            duration_minutes: 0,
            items_moved: 0,
            notes: "아몬드/호두 복합 작업 예정"
          },
          {
            task_id: "TASK-0506-2",
            type: "empty_pallet",
            timestamp: "2024-06-02T11:00:00",
            start_location: "DD-2",
            end_location: "Pallet Storage-9",
            status: "in_progress",
            duration_minutes: 15,
            items_moved: 9,
            notes: "빈 파렛트 1.5개 이동 중"
          }
        ]
      }
    ]
  };
  