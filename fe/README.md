# 버전
node: v22.14.0
npm: 10.9.2

# 설치 순서
## 1단계: Vite + React 프로젝트 생성
npm create vite@latest fe -- --template react
cd fe

## 2단계: 라이브러리 설치
### 1. 라우터,, 주스탠드, 이머, 액시오스, 리액트모달, 헤드리스ui 설치
npm install react-router-dom zustand immer axios react-modal @headlessui/react@latest

### 2. Tailwind CSS v4와 Vite 플러그인 설치
npm install tailwindcss @tailwindcss/vite

### 3. 공식 문서 참고해서 설정
https://tailwindcss.com/docs/installation/using-vite

## 3단계: 프로젝트 실행 및 기본 구조 설정
### 1. npm 설치
npm install

### 2. 폴더구조 및 파일 생성성
```
C:.
│  App.css
│  App.jsx
│  index.css
│  main.jsx
│
├─assets
│      bgimage.png
│      logo.png
│
├─components
│  │  AlertModal.jsx
│  │  ErrorBoundary.jsx
│  │  LoginForm.jsx
│  │  Logout.jsx
│  │  RegisterForm.jsx
│  │
│  ├─atoms
│  │  ├─NavItem
│  │  │      NavItem.css
│  │  │      NavItem.jsx
│  │  │
│  │  └─Toast
│  │          Toast.css
│  │          Toast.jsx
│  │
│  ├─board
│  │      AnnouncementDetail.css
│  │      AnnouncementDetail.jsx
│  │      AnnouncementForm.css
│  │      AnnouncementForm.jsx
│  │      AnnouncementList.css
│  │      AnnouncementList.jsx
│  │
│  ├─Inventory
│  │      InventoryForm.jsx
│  │      InventoryHistory.jsx
│  │      InventoryItem.jsx
│  │      InventoryList.jsx
│  │
│  ├─Layout
│  │      Footer.jsx
│  │      Header.jsx
│  │      Layout.jsx
│  │
│  ├─Map
│  │      MapCanvas.jsx
│  │      MapControls.jsx
│  │      MapInfo.jsx
│  │
│  ├─Robot
│  │      RobotCard.jsx
│  │      RobotDetailsModal.jsx
│  │
│  └─templates
│          .gitkeep
│
├─configs
│      env.js
│
├─hooks
│      useAuth.js
│      useInventory.js
│      useLocation.js
│      useToast.js
│
├─pages
│      InventoryDetailPage.jsx
│      InventoryPage.jsx
│      LoginPage.jsx
│      LogPage.jsx
│      MainPage.jsx
│      MapPage.jsx
│      MapPageOld.jsx
│      RegisterPage.jsx
│      RobotPage.jsx
│      SettingsPage.jsx
│      WorkPage.jsx
│
├─routes
│      AppRoutes.jsx
│
├─services
│      .gitkeep
│      errorService.js
│      inventoryService.js
│
├─stores
│      authStore.js
│      inventoryStore.js
│      toastStore.js
│
├─styles
│      AuthForm.css
│      HeaderStyles.css
│      InventoryStyles.css
│      RobotPage.module.css
│      RobotStyles.css
│
└─utils
        api.js
        mapUtils.js
        robotData.js
```
