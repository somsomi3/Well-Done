# build를 Node 20버전으로 한다.
FROM node:20 AS build
# 작업 디렉토리를 설정한다.
WORKDIR /app
#의존성을 설치하기 위해서 파일을 작업디렉토리로 복사한다.
COPY package-lock.json package.json ./
#의존성 설치하기 >> 개발 중일때는 npm i, 배포하기 직전에는 npm ci 하기
RUN npm i
# 애플리케이션의 소스 파일을 작업 디렉토리로 복사하기
COPY . .
# Vite를 사용해서 정적 파일을 빌드하기
RUN npm run build
# 실제 서비스용 Nginx 이미지 다운받기.
FROM nginx:alpine AS production
# 빌드된 정적 파일을 Nginx 루트 디렉토리로 복사
COPY --from=build /app/dist /usr/share/nginx/html
#기존에 있던 default 설정 파일들을 삭제시키기.
RUN rm /etc/nginx/conf.d/default.conf

COPY nginx/default.conf /etc/nginx/conf.d/default.conf
# Nginx는 기본적으로 포트 80을 사용하니깐 명시적으로 표시하기
EXPOSE 80

CMD ["nginx", "-g", "daemon off;"]


