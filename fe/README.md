# 버전
node: v22.14.0
npm: 10.9.2

# 설치 순서
## 1단계: Vite + React 프로젝트 생성
npm create vite@latest fe -- --template react
cd fe

## 2단계: 라이브러리 설치
### 1. 라우터,, 주스탠드, 이머, 액시오스 설치
npm install react-router-dom zustand immer axios

### 2. Tailwind CSS v4와 Vite 플러그인 설치
npm install tailwindcss @tailwindcss/vite

### 3. 공식 문서 참고해서 설정
https://tailwindcss.com/docs/installation/using-vite

## 3단계: 프로젝트 실행 및 기본 구조 설정
### 1. 프로젝트 실행
npm run dev

### 2. 기초 폴더구조 생성
mkdir -p src/{components,pages,hooks,utils,assets,services,store}

## 4단계: Atomic design 적용
### 1. 폴더 구조 생성
mkdir -p src/components/{atoms,molecules,organisms,templates,pages}

### 2. 아토믹 디자인 가이드
atoms: 기본 UI 요소 (버튼, 인풋 등)

molecules: 여러 atoms의 조합 (폼 입력 등)

organisms: 복잡한 UI 구성 요소 (폼, 헤더 등)

templates: 페이지 레이아웃 템플릿

### 3. 폴더 구조 예시
src/
├── assets/                 # 이미지, 폰트 등 정적 파일
├── components/             # Atomic Design 컴포넌트
│   ├── atoms/              # 기본 UI 요소 (버튼, 인풋, 레이블 등)
│   │   ├── Button/
│   │   │   ├── Button.tsx
│   │   │   └── Button.css
│   │   ├── Input/
│   │   ├── Label/
│   │   └── ...
│   ├── molecules/          # 원자의 조합 (라벨이 있는 인풋, 검색바 등)
│   │   ├── FormInput/
│   │   ├── SearchBar/
│   │   └── ...
│   ├── organisms/          # 분자의 조합 (폼, 헤더, 사이드바 등)
│   │   ├── LoginForm/
│   │   ├── Header/
│   │   ├── Sidebar/
│   │   └── ...
│   ├── templates/          # 페이지 레이아웃 템플릿
│   │   ├── MainTemplate/
│   │   ├── AuthTemplate/
│   │   └── ...
│   └── pages/              # 실제 페이지 컴포넌트
│       ├── auth/
│       │   ├── LoginPage.tsx
│       │   └── RegisterPage.tsx
│       ├── main/
│       │   └── MainPage.tsx
│       ├── notice/
│       │   ├── NoticePage.tsx
│       │   └── NoticeCreatePage.tsx
│       ├── map/
│       │   └── MapPage.tsx
│       ├── robot/
│       │   └── RobotPage.tsx
│       └── inventory/
│           └── InventoryPage.tsx
├── hooks/                  # 커스텀 훅
├── store/                  # Zustand 스토어
│   ├── authStore.ts
│   ├── robotStore.ts
│   ├── inventoryStore.ts
│   └── ...
├── services/               # API 서비스
│   ├── api.ts
│   ├── authService.ts
│   ├── robotService.ts
│   └── ...
├── utils/                  # 유틸리티 함수
├── types/                  # 타입 정의
├── routes/                 # 라우트 설정
│   └── index.tsx
├── App.tsx
├── main.tsx
└── index.css
