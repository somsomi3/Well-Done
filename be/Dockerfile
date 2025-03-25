# 1. 빌드 환경 설정(Gradle 빌드)
#빌드 단계에서 사용하는 기본 이미지 >> Gradle 8.13과 JDK17이 설치된 공식 이미지를 dockerhub에서 가져와서 사용한다.
#AS build는 이 실행 과정을 build라는 이름을 부여해서 나중에 참조해서 사용하기 위해서 하는 설정.
FROM gradle:8.13-jdk17 AS build
#컨테이너 내에서 작업할 디렉토리를 설정하는데 여기서는 /app으로 설정하였다. 디렉토리임을 명확하게 표현하기 위해서 app뒤에 /를 붙임.
WORKDIR /app/
#현재 디렉토리(프로젝트 루트)에 있는 모든 파일을 컨테이너의 /app 디렉토리로 복사한다.
COPY . .
# 프로젝트를 빌드한다. -x test는 테스트는 건너뛴다는 옵션이다.
RUN gradle clean build -x test

# 2. 실행 환경 설정(JAR 실행)
# 첫번째 스테이지에서는 소스 코드를 컴파일하고 JAR 파일을 생성해야 해서 build이미지를 가져와야 하므로 Java컴파일러(JDK), 빌드 도구 (Gradle)이 모두 필요함
# 하지만 여기 두번째 스테이지에서는 이미 빌드된 JAR 파일을 실행하기만 하면 된다.
# JAR 파일 실행에는 JDK나 JRE만 있으면 충분하고, Gradle은 더 이상 필요하지 않습니다.
FROM openjdk:17-jdk-slim
WORKDIR /app
# 컨테이너의 시간대를 한국 시간으로 설정한다.
ENV TZ=Asia/Seoul

#우리가 명시적으로 AS로 했던 build 단계에서 생성된 jar 파일을 app.jar 폴더로 가져온다.
COPY --from=build /app/build/libs/*.jar app.jar
#이 컨테이너가 8080포트를 쓸거라고 문서화해서 알려주는 역할만 할뿐, 실제 포트 매핑은 docker run 명령어나 docker-compose.yml에서 설정한다.
EXPOSE 8080
#CMD 명령어는 컨테이너가 시작될 때 실행할 명령어를 지정한다. java-jar app.jar명령으로 Spring Boot 애플리케이션을 실행한다.
CMD ["java", "-jar", "app.jar"]