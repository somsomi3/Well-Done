import React, { useState, useEffect, useRef } from 'react';
import { useInventory } from '../../hooks/useInventory';
import { useToast } from '../../hooks/useToast';
import { api } from '../../utils/api';
import { useAuthStore } from '../../stores/authStore';
import { getApiUrl } from '../../configs/env';

// 랙 위치 정보 매핑 (하드코딩)
const RACK_LOCATIONS = {
  'A1': { id: 'A1', name: '랙 A1', x: -47.36, y: -63.7, theta: 180 },
  'A2': { id: 'A2', name: '랙 A2', x: -47.89, y: -63.69, theta: 180 },
  'B1': { id: 'B1', name: '랙 B1', x: -51.16, y: -63.69, theta: 180 },
  'B2': { id: 'B2', name: '랙 B2', x: -51.67, y: -63.69, theta: 180 },
  'C1': { id: 'C1', name: '랙 C1', x: -48.73, y: -63.07, theta: 90 },
  'C2': { id: 'C2', name: '랙 C2', x: -48.7, y: -62.58, theta: 90 },
  'D1': { id: 'D1', name: '랙 D1', x: -50.45, y: -63.11, theta: -90 },
  'D2': { id: 'D2', name: '랙 D2', x: -50.45, y: -62.56, theta: -90 },
  'E1': { id: 'E1', name: '랙 E1', x: -47.38, y: -61.98, theta: 0 },
  'E2': { id: 'E2', name: '랙 E2', x: -47.89, y: -61.98, theta: 0 },
  // 필요에 따라 추가 랙 위치를 포함할 수 있습니다
};

// 창고 위치 정보 (제품별)
const WAREHOUSE_LOCATIONS = {
  '쿠크다스': [
    { id: 'RST1', name: '창고 오른쪽 1', x: -60.19, y: -64.82, theta: 90 }
  ],
  '몽쉘': [
    { id: 'LST1', name: '창고 왼쪽 1', x: -59.2, y: -64.82, theta: -90 }
  ]
};

// 빈 파레트 저장소
const EMPTY_PALLET_LOCATION = { id: 'EMP1', name: '빈 파레트 1', x: -55.56, y: -66.42, theta: 0 };

const InventoryItem = ({ item }) => {
  const { adjustInventory, fetchInventory } = useInventory();
  const { success, error } = useToast();
  const { token } = useAuthStore();
  const [adjustAmount, setAdjustAmount] = useState('');
  const [isAdjusting, setIsAdjusting] = useState(false);
  const [isProcessing, setIsProcessing] = useState(false);
  const [processStatus, setProcessStatus] = useState(null);
  
  // 웹소켓 연결 관리
  const socketRef = useRef(null);
  const taskCallbacksRef = useRef({});
  
  // 웹소켓 연결 설정
  useEffect(() => {
    // 컴포넌트가 마운트될 때 웹소켓 연결
    if (token) {
      const apiUrl = getApiUrl();
      const wsUrl = apiUrl.replace('http', 'ws');
      const socket = new WebSocket(`wss://j12e102.p.ssafy.io/ws/user?token=${token}`);
      socketRef.current = socket;
      
      socket.onopen = () => {
        console.log('웹소켓 연결됨');
        // 필요한 경우 join 메시지 전송
        socket.send(JSON.stringify({
          type: 'join'
        }));
      };
      
      socket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          console.log('웹소켓 메시지 수신:', data);
          
          // pick_complete 이벤트 처리
          if (data.type === 'pick_complete') {
            const callback = taskCallbacksRef.current[`pick_${data.from_id.toLowerCase()}`];
            if (callback) {
              if (data.success) {
                callback.resolve(data);
              } else {
                callback.reject(new Error(`물건 집기 실패: ${data.message || '알 수 없는 오류'}`));
              }
              // 콜백 제거
              delete taskCallbacksRef.current[`pick_${data.from_id.toLowerCase()}`];
            }
          }
          // place_complete 이벤트 처리
          else if (data.type === 'place_complete') {
            const callback = taskCallbacksRef.current[`place_${data.to_id.toLowerCase()}`];
            if (callback) {
              if (data.success) {
                callback.resolve(data);
              } else {
                callback.reject(new Error(`물건 놓기 실패: ${data.message || '알 수 없는 오류'}`));
              }
              // 콜백 제거
              delete taskCallbacksRef.current[`place_${data.to_id.toLowerCase()}`];
            }
          }
        } catch (err) {
          console.error('웹소켓 메시지 처리 오류:', err);
        }
      };
      
      socket.onerror = (error) => {
        console.error('웹소켓 오류:', error);
      };
      
      socket.onclose = () => {
        console.log('웹소켓 연결 종료');
      };
    }
    
    // 컴포넌트가 언마운트될 때 웹소켓 연결 종료
    return () => {
      if (socketRef.current && socketRef.current.readyState === WebSocket.OPEN) {
        socketRef.current.close();
      }
    };
  }, [token]);

  const handleAdjustClick = () => {
    setIsAdjusting(true);
  };

  const handleCancelAdjust = () => {
    setIsAdjusting(false);
    setAdjustAmount('');
  };

  const handleAmountChange = (e) => {
    // 숫자만 입력 가능하도록 처리 (음수 허용)
    const value = e.target.value;
    if (value === '' || /^-?\d+$/.test(value)) {
      setAdjustAmount(value);
    }
  };

  // 제품 이름에서 제품 유형 판별 함수
  const getProductType = (itemName) => {
    // 제품명에 '쿠크다스'가 포함되어 있으면 '쿠크다스', 아니면 '몽쉘'로 가정
    if (itemName.includes('쿠크다스')) return '쿠크다스';
    if (itemName.includes('몽쉘')) return '몽쉘';
    
    // 기본값으로 제품명 그대로 반환
    return itemName;
  };
  
  // 웹소켓 이벤트를 기다리는 Promise 생성 함수
  const waitForWebSocketEvent = (eventType, locationId) => {
    const key = `${eventType}_${locationId.toLowerCase()}`;
    
    return new Promise((resolve, reject) => {
      // 이미 등록된 콜백이 있으면 제거
      if (taskCallbacksRef.current[key]) {
        delete taskCallbacksRef.current[key];
      }
      
      // 타임아웃 설정 (2분)
      const timeoutId = setTimeout(() => {
        if (taskCallbacksRef.current[key]) {
          delete taskCallbacksRef.current[key];
          reject(new Error('작업 타임아웃: 2분 이내에 완료되지 않았습니다'));
        }
      }, 120000);
      
      // 콜백 등록
      taskCallbacksRef.current[key] = {
        resolve: (data) => {
          clearTimeout(timeoutId);
          resolve(data);
        },
        reject: (error) => {
          clearTimeout(timeoutId);
          reject(error);
        }
      };
    });
  };

  // 창고에서 물건 보충 프로세스
  const replenishFromWarehouse = async (rackId) => {
    if (!token) {
      error('인증 토큰이 없습니다.');
      return;
    }
    
    // 웹소켓 연결 확인
    if (!socketRef.current || socketRef.current.readyState !== WebSocket.OPEN) {
      error('웹소켓 연결이 없습니다. 페이지를 새로고침 후 다시 시도해주세요.');
      return;
    }

    setIsProcessing(true);
    setProcessStatus('시작: 창고에서 물건 보충 프로세스');

    try {
      // 1. 창고에 재고가 있는지 확인
      if (item.warehouse_quantity <= 0) {
        setProcessStatus('❌ 창고에 재고가 없습니다.');
        error('창고에 재고가 없습니다.');
        setTimeout(() => setIsProcessing(false), 3000);
        return;
      }

      // DB에서 가져온 물건 이름으로 제품 유형 판별
      const productType = getProductType(item.item_name);
      
      // 랙 위치 확인
      const rackLocation = RACK_LOCATIONS[rackId];
      if (!rackLocation) {
        setProcessStatus(`❌ ${rackId} 위치 정보를 찾을 수 없습니다.`);
        error(`${rackId} 위치 정보를 찾을 수 없습니다.`);
        setTimeout(() => setIsProcessing(false), 3000);
        return;
      }

      // 창고 위치 확인 (첫 번째 위치 사용)
      const warehouseLocations = WAREHOUSE_LOCATIONS[productType];
      if (!warehouseLocations || warehouseLocations.length === 0) {
        setProcessStatus(`❌ ${productType} 창고 위치 정보를 찾을 수 없습니다.`);
        error(`${productType} 창고 위치 정보를 찾을 수 없습니다.`);
        setTimeout(() => setIsProcessing(false), 3000);
        return;
      }
      
      // 첫 번째 창고 위치 사용
      const storageLocation = warehouseLocations[0];

      // 2. 빈 파레트를 임시 저장소로 이동
      setProcessStatus(`🚚 1단계: ${rackId}에서 빈 파레트 제거 중...`);
      const removeEmptyCommand = {
        from: {
          x: rackLocation.x,
          y: rackLocation.y,
          theta: rackLocation.theta,
        },
        to: {
          x: EMPTY_PALLET_LOCATION.x,
          y: EMPTY_PALLET_LOCATION.y,
          theta: EMPTY_PALLET_LOCATION.theta,
        },
        product_id: item.item_name, // DB에 저장된 물건 이름 사용
        from_id: rackId,
        to_id: EMPTY_PALLET_LOCATION.id,
      };

      // 첫 번째 작업의 place_complete 이벤트를 위한 Promise 생성
      const firstTaskPromise = waitForWebSocketEvent('place', EMPTY_PALLET_LOCATION.id);
      
      // 빈 파레트 제거 명령 전송
      console.log('빈 파레트 제거 명령 전송:', removeEmptyCommand);
      await api.post('/robot/pick-place', removeEmptyCommand);
      
      // place_complete 이벤트 대기
      setProcessStatus('⏱️ 1단계: 빈 파레트 이동 완료 대기 중...');
      await firstTaskPromise;
      
      setProcessStatus('✅ 1단계: 빈 파레트 제거 완료');
      
      // 3. 창고에서 새 물건을 랙으로 이동
      setProcessStatus(`🚚 2단계: 창고에서 ${rackId}로 새 파레트 이동 중...`);
      const addNewCommand = {
        from: {
          x: storageLocation.x,
          y: storageLocation.y,
          theta: storageLocation.theta,
        },
        to: {
          x: rackLocation.x,
          y: rackLocation.y,
          theta: rackLocation.theta,
        },
        product_id: item.item_name, // DB에 저장된 물건 이름 사용
        from_id: storageLocation.id,
        to_id: rackId,
      };

      // 두 번째 작업의 place_complete 이벤트를 위한 Promise 생성
      const secondTaskPromise = waitForWebSocketEvent('place', rackId);
      
      // 새 파레트 추가 명령 전송
      console.log('새 파레트 추가 명령 전송:', addNewCommand);
      await api.post('/robot/pick-place', addNewCommand);
      
      // place_complete 이벤트 대기
      setProcessStatus('⏱️ 2단계: 새 파레트 이동 완료 대기 중...');
      await secondTaskPromise;
      
      setProcessStatus('✅ 2단계: 새 파레트 추가 완료');
      
      // 4. 인벤토리 데이터 업데이트
      const updateAmount = Math.min(item.warehouse_quantity, 10); // 최대 10개 가정 또는 창고 수량
      await adjustInventory(item.id, updateAmount);
      
      // 창고 수량 감소 - 서버에서 지원하는 API가 있다면 사용
      try {
        await api.post(`/inventory/${item.id}/warehouse-adjust`, {
          amount: -updateAmount
        });
      } catch (err) {
        console.warn('창고 수량 조정 API가 없거나 오류 발생:', err);
        // 창고 수량 조정 API가 없을 경우 메시지만 출력하고 계속 진행
      }

      // 5. 인벤토리 데이터 새로고침
      await fetchInventory();

      setProcessStatus(`✅ 완료: ${rackId}에 ${item.item_name} ${updateAmount}개 보충됨`);
      success(`${item.item_name}이(가) 창고에서 ${updateAmount}개 보충되었습니다.`);
      
      // 3초 후 상태 초기화
      setTimeout(() => {
        setIsProcessing(false);
        setProcessStatus(null);
      }, 3000);
      
    } catch (err) {
      console.error('보충 프로세스 오류:', err);
      setProcessStatus(`❌ 오류: ${err.message}`);
      error(`보충 프로세스 중 오류가 발생했습니다: ${err.message}`);
      
      // 작업 실패 시 재고 데이터 새로고침
      try {
        await fetchInventory();
      } catch (refreshErr) {
        console.error('실패 후 데이터 새로고침 오류:', refreshErr);
      }
      
      setTimeout(() => setIsProcessing(false), 3000);
    }
  };

  const handleSubmitAdjust = async () => {
    if (adjustAmount === '') {
      error('수량을 입력해주세요.');
      return;
    }

    const adjustValue = parseInt(adjustAmount, 10);
    
    try {
      // 현재 수량과 조정량 계산
      const currentAmount = item.quantity;
      const newAmount = currentAmount + adjustValue;
      
      // 수량 조정 실행
      await adjustInventory(item.id, adjustValue);
      success(`${item.item_name}의 수량이 ${adjustValue > 0 ? '증가' : '감소'}되었습니다.`);
      
      // 인벤토리 데이터 새로고침
      await fetchInventory();
      
      // 조정 후 수량이 0이 되면 자동 보충 프로세스 시작
      if (newAmount === 0 && item.warehouse_quantity > 0) {
        const rackId = await showRackSelectionDialog(item.item_name);
        if (rackId) {
          // 사용자가 랙 위치를 선택했을 때만 보충 프로세스 시작
          await replenishFromWarehouse(rackId);
        }
      }
      
      setIsAdjusting(false);
      setAdjustAmount('');
    } catch (err) {
      error(`수량 조정 중 오류가 발생했습니다: ${err.message}`);
    }
  };

  // 랙 위치 선택 다이얼로그 표시
  const showRackSelectionDialog = async (productName) => {
    // 실제 구현에서는 UI 다이얼로그를 표시하고 사용자 입력을 받아야 함
    // 여기서는 간단히 prompt로 구현
    return prompt(
      `${productName}의 수량이 0이 되었습니다. 창고에서 보충할 랙 위치를 선택하세요 (A1, A2, B1 등):`,
      "A1"
    );
  };

  return (
    <tr className="inventory-item">
      <td>{item.id}</td>
      <td>{item.item_name}</td>
      <td className={item.quantity === 0 ? "text-red-600 font-bold" : ""}>{item.quantity}</td>
      <td>{item.warehouse_quantity}</td>
      <td>{item.min_threshold}</td>
      <td>
        {isProcessing ? (
          <div className="process-status p-2 bg-blue-100 rounded text-sm">
            {processStatus}
          </div>
        ) : isAdjusting ? (
          <div className="adjust-controls">
            <input
              type="text"
              value={adjustAmount}
              onChange={handleAmountChange}
              placeholder="±수량"
              className="adjust-input"
            />
            <button
              onClick={handleSubmitAdjust}
              className="adjust-button confirm"
            >
              확인
            </button>
            <button
              onClick={handleCancelAdjust}
              className="adjust-button cancel"
            >
              취소
            </button>
          </div>
        ) : (
          <div className="flex space-x-2">
            <button
              onClick={handleAdjustClick}
              className="adjust-button"
            >
              수량 조정
            </button>
            {item.quantity === 0 && item.warehouse_quantity > 0 && (
              <button 
                onClick={() => replenishFromWarehouse('D2')}
                className="replenish-button bg-green-500 text-white px-2 py-1 rounded text-sm"
              >
                보충
              </button>
            )}
          </div>
        )}
      </td>
    </tr>
  );
};

export default InventoryItem;