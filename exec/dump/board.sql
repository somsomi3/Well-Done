-- MySQL dump 10.13  Distrib 8.0.40, for Win64 (x86_64)
--
-- Host: j12e102.p.ssafy.io    Database: welldone
-- ------------------------------------------------------
-- Server version	8.0.41

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `board`
--

DROP TABLE IF EXISTS `board`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `board` (
  `view_count` int NOT NULL,
  `created_at` datetime(6) NOT NULL,
  `expiration_date` datetime(6) DEFAULT NULL,
  `id` bigint NOT NULL AUTO_INCREMENT,
  `updated_at` datetime(6) NOT NULL,
  `content` text NOT NULL,
  `title` varchar(255) NOT NULL,
  `writer` varchar(255) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=38 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `board`
--

LOCK TABLES `board` WRITE;
/*!40000 ALTER TABLE `board` DISABLE KEYS */;
INSERT INTO `board` VALUES (17,'2025-04-09 08:16:02.293327','2025-04-15 14:15:00.000000',19,'2025-04-09 08:19:09.465237','내용: 안녕하세요, 물류팀입니다.\n\n우리 물류 시스템의 핵심인 자율 주행 로봇의 배터리 점검이 다가오는 **2025년 4월 15일(화)**에 예정되어 있습니다. 이 점검은 로봇들의 지속적인 효율성과 안정적인 운행을 보장하기 위한 중요한 절차입니다.\n\n점검 일정 및 주요 사항:\n\n점검 날짜: 2025년 4월 15일(화)부터 시작하여 2025년 4월 16일(수)까지 진행됩니다.\n\n점검 대상: 모든 자율 주행 로봇 (운행 중인 로봇 및 대기 중인 로봇 포함)\n\n점검 시간: 점검은 오전 9시부터 오후 6시까지 진행됩니다. 이 시간 동안 일부 로봇은 작동을 중단하고 점검을 받을 예정입니다.\n\n배터리 점검 항목:\n\n배터리 충전 상태: 각 로봇의 배터리 충전 상태를 점검하여 충전 능력을 테스트합니다. 충전 불량이 발견된 경우 즉시 배터리 교체가 이루어집니다.\n\n배터리 수명: 각 로봇의 배터리 수명을 확인하고, 수명이 다한 배터리는 교체 대상에 포함됩니다.\n\n배터리 연결 점검: 배터리와 로봇 간의 연결 상태를 점검하여 접촉 불량 등의 문제를 예방합니다.\n\n긴급 정지 시스템 테스트: 배터리 관련 긴급 정지 시스템이 정상적으로 작동하는지 확인합니다.\n\n점검 후 조치 사항:\n\n점검이 완료된 후, 배터리 교체가 필요한 로봇은 즉시 교체될 예정이며, 교체 작업 후에는 새로운 배터리로 전환되어 더욱 긴 운행 시간을 보장합니다.\n\n점검이 완료된 로봇은 정상 운행을 시작할 수 있습니다. 각 로봇의 운행 가능 시간은 시스템에서 확인할 수 있으며, 점검 후 결과에 따라 일부 로봇은 운행을 재개하지 못할 수 있습니다.\n\n점검 중의 주의 사항:\n\n점검 동안 일부 로봇은 로딩 구역 및 작업 구역에서 벗어나 점검 공간으로 이동될 수 있습니다. 이로 인해 물류 흐름에 약간의 지연이 발생할 수 있습니다. 이에 대한 이해와 협조 부탁드립니다.\n\n배터리 교체 작업 시에는 특별히 주의하여 안전하게 작업을 진행합니다. 작업 중 발생할 수 있는 불편을 최소화할 수 있도록 최선을 다하겠습니다.\n\n점검 후 성능 개선: 배터리 점검 및 교체를 통해 자율 주행 로봇의 성능이 크게 개선될 것으로 예상됩니다. 특히, 배터리 용량 증가로 인해 로봇의 운행 시간이 더 길어지고, 운행 안정성이 강화될 것입니다. 이를 통해 물류 작업의 효율성도 한층 더 향상될 것입니다.\n\n문의 사항: 점검에 대해 궁금한 사항이나 추가로 필요한 정보가 있으면 물류팀 지원부서로 문의해 주시기 바랍니다. 빠른 시간 내에 답변드리도록 하겠습니다.\n\n이 점검이 우리 시스템의 안정성 향상과 물류 운영 효율성 증대에 큰 도움이 될 것입니다. 직원 여러분의 적극적인 협조를 부탁드리며, 불편함이 최소화될 수 있도록 최선을 다하겠습니다.\n\n감사합니다.\n\n물류팀 드림','로봇 배터리 점검 안내','admin'),(3,'2025-04-09 08:17:13.770008','2025-04-29 23:20:00.000000',21,'2025-04-09 08:17:13.770014','물류 장비 점검이 4월 14일에 진행됩니다. 점검 전 장비 점검을 완료해 주세요.','물류 장비 점검 안내','admin'),(1,'2025-04-09 08:29:34.013392','2025-04-15 23:29:00.000000',24,'2025-04-09 08:29:34.013399','4월 10일 02:00부터 04:00까지 서버 점검이 진행됩니다. 점검 시간 동안 서비스 이용에 제한이 있을 수 있습니다','물류 시스템 서버 점검 안내','admin'),(21,'2025-04-09 08:30:44.662499','2025-04-15 05:30:00.000000',28,'2025-04-09 08:47:52.817514','**제목: 물류 차량 점검 일정 안내**\n\n**내용**:\n안녕하세요, 물류팀입니다.\n\n우리는 물류 차량의 정기적인 점검을 통해 운행 안전과 효율성을 유지하고 있습니다. 다가오는 2025년 4월 20일에 예정된 물류 차량의 점검 일정에 대해 안내드리니, 모든 직원 분들은 참고하시어 업무에 지장이 없도록 준비해 주시기 바랍니다.\n\n**1. 점검 일정**\n점검 날짜: 2025년 4월 20일(월)\n\n점검 시간: 오전 9시부터 오후 6시까지\n\n점검 대상: 모든 물류 차량 (트럭, 밴 등)\n\n점검 중 일부 차량은 운행 불가 상태가 되며, 해당 차량은 점검 후 즉시 운행을 재개할 수 있습니다.\n\n**2. 점검 항목**\n점검 중에는 다음과 같은 주요 항목들이 점검됩니다:\n\n엔진 상태: 엔진 성능 및 오일 점검\n\n타이어 점검: 타이어 마모 상태 및 공기압 점검\n\n배터리 점검: 배터리 충전 상태 및 수명 점검\n\n브레이크 시스템: 브레이크 성능 점검 및 이상 유무 확인\n\n배출가스 검사: 환경 기준에 맞는 배출가스 수준 확인\n\n차체 및 전장 장비 점검: 차량 외부 및 내부 장비 점검 (라이트, 히터, 에어컨 등)\n\n**3. 점검 중의 주의 사항**\n점검이 진행되는 동안 일부 차량은 운행 불가 상태가 될 수 있습니다.\n\n점검 중인 차량은 작업 구역으로 이동할 예정이므로, 물류 흐름에 지장이 생길 수 있습니다.\n\n긴급 배송이 필요한 경우, 다른 운송 수단을 통해 배송을 진행할 수 있도록 미리 준비해 주세요.\n\n**4. 점검 후 조치 사항**\n점검 후 교체가 필요한 부품은 즉시 교체 작업을 진행할 예정입니다.\n\n교체된 부품은 새로운 부품으로 교체되어 차량의 운행 효율성과 안전성이 향상됩니다.\n\n점검이 완료된 차량은 점검 결과에 따라 즉시 운행을 재개할 수 있으며, 점검이 완료된 차량의 상태는 내부 시스템을 통해 확인할 수 있습니다.\n\n**5. 불편 사항에 대한 안내**\n점검 중 일부 차량의 운행이 중단될 수 있으므로, 해당 시간대에 배송 지연이 발생할 수 있습니다.\n\n불편을 최소화하기 위해 점검이 진행되는 동안 배송 우선순위를 조정할 수 있습니다.\n\n기타 문의 사항이나 불편한 점은 물류팀으로 문의해 주세요.\n\n**6. 문의 사항**\n점검에 대해 궁금한 사항이나 추가적인 정보가 필요하시면, 물류팀 지원부서로 문의해 주세요.\n\n문의처: 물류팀 지원부서\n\n전화: 010-1234-5678\n\n이메일: logistics@company.com\n\n이번 점검을 통해 차량의 안전성과 운행 효율성이 개선되어, 물류 서비스의 품질이 한층 더 향상될 것입니다. 직원 여러분의 협조를 부탁드리며, 불편함이 최소화될 수 있도록 최선을 다하겠습니다.\n\n감사합니다.\n\n물류팀 드림','물류 차량 점검 일정 안내','admin'),(2,'2025-04-10 11:37:23.491380','2025-04-17 02:37:00.000000',31,'2025-04-10 11:37:23.491387','안녕하세요.\n현재 내부 물류 시스템에 일시적인 장애가 발생하여 서비스 이용에 불편을 드리고 있습니다.\n장애 원인은 파악 중이며, 최대한 빠르게 정상화될 수 있도록 조치 중입니다.\n\n🔧 영향 범위\n\n물류 현황 조회 지연\n\n배송 상태 실시간 반영 오류\n\n🕒 조치 예정 시간\n\n2025년 4월 10일(목) 오후 3시까지 복구 예정','물류 시스템 장애 발생 안내','admin'),(1,'2025-04-10 11:38:41.247795','2025-05-22 02:37:00.000000',32,'2025-04-10 11:38:41.247801','물류 직원 대상 안전 교육이 4월 15일(화) 오전 10시에 물류센터 1층 교육장에서 진행됩니다.\n전 직원 필참이며, 안전 수칙 및 위기 대응 요령을 교육할 예정입니다.','물류 직원 안전 교육 공지','admin'),(8,'2025-04-10 11:40:20.129459','2025-04-17 02:40:00.000000',33,'2025-04-10 11:40:20.129466','모든 부서에서 주간 물류 보고서를 금요일 오전까지 제출해 주세요.','주간 물류 보고서 제출 안내','admin'),(0,'2025-04-11 01:09:27.172797','2025-04-17 16:09:00.000000',37,'2025-04-11 01:09:27.172805','asd','asd','admin');
/*!40000 ALTER TABLE `board` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2025-04-11 11:17:38
