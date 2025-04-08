// 좌표 변환 유틸리티
const MAP_WIDTH = 480;
const MAP_HEIGHT = 480;
const MAP_RESOLUTION = 0.05;
const ORIGIN_X = -64.5;
const ORIGIN_Y = -71.0;

export const robotToPixelCoordinates = (robotX, robotY) => {
  const stdPixelX = Math.floor((robotX - ORIGIN_X) / MAP_RESOLUTION);
  const stdPixelY = MAP_HEIGHT - Math.floor((robotY - ORIGIN_Y) / MAP_RESOLUTION);
  
  // 180도 회전 적용 (x, y) -> (-x, -y) -> (width-x, height-y)
  const pixelX = MAP_WIDTH - stdPixelX;
  const pixelY = MAP_HEIGHT - stdPixelY;
  
  console.log(`변환 과정: 원본(${stdPixelX},${stdPixelY}) -> 180도 회전(${pixelX},${pixelY})`);
  
  return { pixelX, pixelY };
};

export const pixelToRobotCoordinates = (pixelX, pixelY) => {
  // 180도 회전 역변환 (동일하게 180도 회전)
  const stdPixelX = MAP_WIDTH - pixelX;
  const stdPixelY = MAP_HEIGHT - pixelY;
  
  // 로봇 좌표 계산
  const robotX = (stdPixelX * MAP_RESOLUTION) + ORIGIN_X;
  const robotY = ((MAP_HEIGHT - stdPixelY) * MAP_RESOLUTION) + ORIGIN_Y;
  
  console.log(`역변환 과정: 입력(${pixelX},${pixelY}) -> 180도 역회전(${stdPixelX},${stdPixelY}) -> 로봇(${robotX.toFixed(2)},${robotY.toFixed(2)})`);
  
  return { robotX, robotY };
};
