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
-- Table structure for table `inventory_history`
--

DROP TABLE IF EXISTS `inventory_history`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `inventory_history` (
  `id` bigint NOT NULL AUTO_INCREMENT,
  `created_at` datetime(6) DEFAULT NULL,
  `updated_at` datetime(6) DEFAULT NULL,
  `action_type` varchar(255) NOT NULL,
  `change_amount` int NOT NULL,
  `item_name` varchar(255) NOT NULL,
  `min_threshold` int NOT NULL,
  `remark` varchar(255) DEFAULT NULL,
  `result_quantity` int NOT NULL,
  `updated_by` varchar(255) NOT NULL,
  `warehouse_total` int NOT NULL,
  `inventory_id` bigint DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKlh93unaa26ttn4ey42m8r31nd` (`inventory_id`),
  CONSTRAINT `FKlh93unaa26ttn4ey42m8r31nd` FOREIGN KEY (`inventory_id`) REFERENCES `inventory` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=43 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `inventory_history`
--

LOCK TABLES `inventory_history` WRITE;
/*!40000 ALTER TABLE `inventory_history` DISABLE KEYS */;
INSERT INTO `inventory_history` VALUES (1,'2025-04-10 07:33:21.223323','2025-04-10 07:33:21.223323','DECREASE',-20,'빼빼로',0,'판매',30,'com.be.domain.auth.dto.UserDto@671d00b6',0,1),(2,'2025-04-10 07:34:11.365174','2025-04-10 07:34:11.365174','DECREASE',-30,'빼빼로',0,'판매',0,'com.be.domain.auth.dto.UserDto@7e1d109e',0,1),(3,'2025-04-10 07:35:12.880321','2025-04-10 07:35:12.880321','INCREASE',20,'빼빼로',0,'입고',20,'com.be.domain.auth.dto.UserDto@11d5565e',0,1),(4,'2025-04-10 07:35:50.348064','2025-04-10 07:35:50.348064','INCREASE',30,'빼빼로',0,'입고',50,'com.be.domain.auth.dto.UserDto@7d802468',0,1),(5,'2025-04-10 07:41:21.172994','2025-04-10 07:41:21.172994','DECREASE',-20,'빼빼로',0,'판매',30,'com.be.domain.auth.dto.UserDto@1a74d8c9',0,1),(6,'2025-04-10 07:41:44.248563','2025-04-10 07:41:44.248563','INCREASE',50,'쿠크다스',0,'입고',100,'com.be.domain.auth.dto.UserDto@63908c66',250,7),(7,'2025-04-10 07:42:00.264233','2025-04-10 07:42:00.264233','DECREASE',-100,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@77fb0573',250,7),(8,'2025-04-10 07:42:39.958761','2025-04-10 07:42:39.958761','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@37f846bf',250,7),(9,'2025-04-10 07:43:04.060741','2025-04-10 07:43:04.060741','DECREASE',-10,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@2fec69f1',250,7),(10,'2025-04-10 07:51:50.314156','2025-04-10 07:51:50.314156','DECREASE',-30,'오뜨',0,'판매',20,'com.be.domain.auth.dto.UserDto@7857575',0,6),(11,'2025-04-10 07:52:13.691717','2025-04-10 07:52:13.691717','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@c243c12',250,7),(12,'2025-04-10 08:02:05.840639','2025-04-10 08:02:05.840639','DECREASE',-10,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@54852a71',250,7),(13,'2025-04-10 08:04:12.734299','2025-04-10 08:04:12.734299','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@34dcea82',250,7),(14,'2025-04-10 08:12:46.027234','2025-04-10 08:12:46.027234','INCREASE',10,'쿠크다스',0,'입고',20,'com.be.domain.auth.dto.UserDto@5eed03a3',250,7),(15,'2025-04-10 08:12:59.282643','2025-04-10 08:12:59.282643','DECREASE',-20,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@6ca1addb',250,7),(16,'2025-04-10 08:13:50.712793','2025-04-10 08:13:50.712793','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@2f9e2325',250,7),(17,'2025-04-10 08:24:12.869369','2025-04-10 08:24:12.869369','DECREASE',-10,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@11de1bd3',250,7),(18,'2025-04-10 08:30:17.110932','2025-04-10 08:30:17.110932','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@d64acb6',250,7),(19,'2025-04-10 08:30:33.653385','2025-04-10 08:30:33.653385','DECREASE',-10,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@6c584f0e',250,7),(20,'2025-04-10 08:34:09.208953','2025-04-10 08:34:09.208953','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@3f9118e1',250,7),(21,'2025-04-10 08:34:39.956677','2025-04-10 08:34:39.956677','DECREASE',-10,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@2a9a6ed2',250,7),(22,'2025-04-10 08:36:02.331331','2025-04-10 08:36:02.331331','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@2d802fb4',250,7),(23,'2025-04-10 20:53:47.807812','2025-04-10 20:53:47.807812','INCREASE',25,'빼빼로',0,'입고',55,'com.be.domain.auth.dto.UserDto@53358242',0,1),(24,'2025-04-10 12:36:32.813119','2025-04-10 12:36:32.813119','DECREASE',-10,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@6ccc054',250,7),(25,'2025-04-10 12:38:39.411325','2025-04-10 12:38:39.411325','DECREASE',-50,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@2e95ca7f',250,8),(26,'2025-04-10 12:42:37.726308','2025-04-10 12:42:37.726308','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@40c34d69',250,7),(27,'2025-04-10 12:42:40.418672','2025-04-10 12:42:40.418672','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@393a7393',250,8),(28,'2025-04-10 13:57:31.155535','2025-04-10 13:57:31.155535','INCREASE',25,'쿠크다스',0,'입고',35,'com.be.domain.auth.dto.UserDto@59f79ffe',250,7),(29,'2025-04-10 13:57:38.308771','2025-04-10 13:57:38.308771','DECREASE',0,'쿠크다스',0,'판매',35,'com.be.domain.auth.dto.UserDto@65e20cd8',250,7),(30,'2025-04-10 13:57:57.047089','2025-04-10 13:57:57.047089','INCREASE',35,'쿠크다스',0,'입고',70,'com.be.domain.auth.dto.UserDto@2d417645',250,7),(31,'2025-04-10 13:58:44.372535','2025-04-10 13:58:44.372535','DECREASE',-70,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@47821d5',250,7),(32,'2025-04-10 15:00:09.821170','2025-04-10 15:00:09.821170','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@c4e6061',250,7),(33,'2025-04-10 15:20:49.925602','2025-04-10 15:20:49.925602','DECREASE',-10,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@1fecccf7',250,7),(34,'2025-04-10 15:22:54.299576','2025-04-10 15:22:54.299576','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@79ea235',250,7),(35,'2025-04-10 18:57:35.988652','2025-04-10 18:57:35.988652','DECREASE',-10,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@3f6b0b42',250,8),(36,'2025-04-10 18:59:54.765250','2025-04-10 18:59:54.765250','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@44868d9b',250,8),(37,'2025-04-10 19:00:42.200215','2025-04-10 19:00:42.200215','DECREASE',-10,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@b832d6',250,8),(38,'2025-04-10 19:02:02.647635','2025-04-10 19:02:02.647635','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@6c6bd7de',250,8),(39,'2025-04-10 23:56:30.872882','2025-04-10 23:56:30.872882','DECREASE',-10,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@6a2f7f1a',250,8),(40,'2025-04-10 23:58:21.573649','2025-04-10 23:58:21.573649','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@599ab617',250,8),(41,'2025-04-11 01:11:11.850555','2025-04-11 01:11:11.850555','DECREASE',-10,'쿠크다스',0,'판매',0,'com.be.domain.auth.dto.UserDto@705a5ad5',250,7),(42,'2025-04-11 01:12:36.666279','2025-04-11 01:12:36.666279','INCREASE',10,'쿠크다스',0,'입고',10,'com.be.domain.auth.dto.UserDto@703e4ef4',250,7);
/*!40000 ALTER TABLE `inventory_history` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2025-04-11 11:27:12
