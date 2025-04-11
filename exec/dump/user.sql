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
-- Table structure for table `user`
--

DROP TABLE IF EXISTS `user`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `user` (
  `created_at` datetime(6) DEFAULT NULL,
  `id` bigint NOT NULL AUTO_INCREMENT,
  `updated_at` datetime(6) DEFAULT NULL,
  `username` varchar(100) NOT NULL,
  `company_id` varchar(255) NOT NULL,
  `email` varchar(255) NOT NULL,
  `password` varchar(255) NOT NULL,
  `user_id` varchar(255) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `UKob8kqyqqgmefl0aco34akdtpe` (`email`),
  UNIQUE KEY `UKa3imlf41l37utmxiquukk8ajc` (`user_id`)
) ENGINE=InnoDB AUTO_INCREMENT=17 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `user`
--

LOCK TABLES `user` WRITE;
/*!40000 ALTER TABLE `user` DISABLE KEYS */;
INSERT INTO `user` VALUES ('2025-04-04 06:08:19.478644',1,'2025-04-04 06:08:19.478644','admin','00000000','admin@example.com','$2a$10$gz5RreDdTxW44dRYojng7eOfQGfU5XsRP4imBETqGvKYZt0byaAK6','675dfca2-feed-4876-a3dc-cae5e40ff162'),('2025-04-04 06:15:51.603959',2,'2025-04-04 06:15:51.603959','test2','test2','test2@test2.com','$2a$10$uUxdU0p/3DkSwowyr7iwJO6las3l7YIE.icL2P9XxZ1Y9KW5YPVo2','0f6506e0-9208-42ff-a927-5eca7957c2e6'),('2025-04-04 06:21:40.379803',3,'2025-04-04 06:21:40.379803','robot','asd','robot@robot.com','$2a$10$MFwL/D3aDvBPmGD36bSx1u673xCxRvsnHteRn250O6.feftGzbLWq','1e179f10-7322-4664-8bc7-4457be15042e'),('2025-04-04 06:23:24.266964',4,'2025-04-04 06:23:24.266964','asdasd','0000000','asdasd@asd.com','$2a$10$9/Tb3qJ.6lnZJIOm5scb8utb6xM3bsBmWqm94G1ysKOXE.3X7yENy','0ca3f85c-8256-4111-ab91-b788a4311c15'),('2025-04-04 07:02:19.576983',5,'2025-04-04 07:02:19.576983','ssafy1','ssafy1','ssafy1@ssafy1.ssafy1','$2a$10$rdeLSn6nyrJN2kbsBMDZnehJR3/8adj6s3FuoNPVYyI0mBKKuNSzC','299fdc88-91fa-4d3a-92ee-a2a460f800b8'),('2025-04-04 08:09:20.887683',6,'2025-04-04 08:09:20.887683','ssafy2','ssafy2','ssafy2@ssafy2.ssafy2','$2a$10$a1bc3LPvSgZ40vcgu8GzIeBi5eGQtNaNeohiAYELbd4IqBlxK4.2O','9340d183-a5c0-480d-a615-6cf707424a88'),('2025-04-06 06:03:55.011480',7,'2025-04-06 06:03:55.011480','test','111111','test@test.com','$2a$10$qb2FNkD/ayR1nfCuLxqvT.tzhk.7JV9djAQfUEMM.ljXHZl2l6DJm','d7ae7f08-b7ef-4ecb-83f9-e7335b4e90a9'),('2025-04-07 00:52:22.896934',8,'2025-04-07 00:52:22.896934','ssafy3','ssafy3','ssafy3@ssafy3.ssafy3','$2a$10$WXKPhzni2gArEX5OAv89N.B3P.CiWMunS0v7ZeDZxi6jRSEKRhImW','02708819-9b10-470d-96de-90c373b8f6b6'),('2025-04-07 00:55:22.425544',9,'2025-04-07 00:55:22.425544','test1','11111','test1@test1.com','$2a$10$cP8LxJUNdNLDsd7mFDOkf.REPXQ.gdzLjk8Ar68gckGv.KFoBemW2','7cd1f42a-bbeb-4eae-acd0-248b3374799a'),('2025-04-07 01:48:44.772583',10,'2025-04-07 01:48:44.772583','3333','kk','test@naver.com','$2a$10$3zvWE2OCeUPKORNOzTNFVOElfR09Mbha1qpthGYiJWiuanvqqJPde','d2539143-5792-4ed8-9a06-b9c85e57c83d'),('2025-04-07 01:55:53.998299',11,'2025-04-07 01:55:53.998299','ssafy4','ssafy4','ssafy4@ssafy4.ssafy4','$2a$10$to95HHwXvvQleRwxVtws/e3Psak7kmECjKPooZE5812fReeyp8dR2','b08a7daa-3488-4f86-9bae-83110a7a870c'),('2025-04-08 07:13:40.675451',12,'2025-04-08 07:13:40.675451','ehdgus8764','01357','ehdgus8764@gmail.com','$2a$10$aVqYqHBlIrme//ssYp5lAeDWZr5cQdqfgf1T3JxmaxAdiFoBaTKXm','3b117ea7-ae82-42d2-a0fe-0401b094d887'),('2025-04-08 16:47:44.474294',13,'2025-04-08 16:47:44.474294','edwin8764','asodjaoisd','edwin.dkim@gmail.com','$2a$10$j.izxiZssIPj9.OORHNFHu/Cqq9g.m8SyuOtKB2DO8U3PCBM6LijC','b79d741c-d618-48ea-a6a4-8af82ceb2cc4'),('2025-04-10 13:47:02.065240',14,'2025-04-10 13:47:02.065240','edwin4947','01357','edwin@gmail.com','$2a$10$oZGqAVFbxs8wq1GhcWkEKul9rHeJWGsp3HmrG78swdYTpg/Q6JD1e','d966e166-45bf-4b72-806f-d6a7c5a5a9e2'),('2025-04-10 13:49:24.413755',15,'2025-04-10 13:49:24.413755','sky1357','1234','sky1357@gmail.com','$2a$10$2uxILpGm8M7dqPXrb.au3ezZvVyXrvSLOFFn/yWvgI0B127C2DF16','b4025ed7-7302-4d37-bb53-20b91b9e61f3'),('2025-04-11 00:56:44.813134',16,'2025-04-11 00:56:44.813134','dd123','23','123@123','$2a$10$8FH3EqXvMbudYrx6LdMxwu/xhr/UHlAQCTk9VXxmGk6e1PSJtS1CK','3f73038a-1d01-48df-9c19-82c56d56d5db');
/*!40000 ALTER TABLE `user` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2025-04-11 11:33:09
