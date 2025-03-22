package com.be.connection.mqtt.config;

import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.integration.annotation.MessagingGateway;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.integration.channel.DirectChannel;
import org.springframework.integration.mqtt.core.DefaultMqttPahoClientFactory;
import org.springframework.integration.mqtt.core.MqttPahoClientFactory;
import org.springframework.integration.mqtt.outbound.MqttPahoMessageHandler;
import org.springframework.messaging.MessageChannel;

import jakarta.websocket.MessageHandler;

/**
 * MQTT 관련 설정을 관리하는 클래스입니다.
 */
@Configuration
public class MqttConfig {

	@Value("${spring.mqtt.broker-url}")//mqtt 보내기위한 브로커 서버 url - 추후 수정 필요
	private String brokerUrl;

	@Value("${spring.mqtt.client-id}")//mqtt 클라이언트의 고유 아이디 id
	private String clientId;

	@Value("${spring.mqtt.topic}")//MQTT 메시지를 발행할 토픽입니다. 여러 클라이언트가 동일한 토픽을 구독할 수 있습니다.
	private String topic;

	// MQTT 클라이언트 팩토리 빈 생성 . 해당 팩토리는 MQTT 클라이언트 연결 옵션을 설정합니다.
	@Bean
	public MqttPahoClientFactory mqttClientFactory() {
		DefaultMqttPahoClientFactory factory = new DefaultMqttPahoClientFactory();
		MqttConnectOptions options = new MqttConnectOptions();
		options.setServerURIs(new String[]{brokerUrl});//MQTT 브로커 서버 주소
		factory.setConnectionOptions(options);//연결 옵션 설정 관련
		return factory;
	}

	/**
	 * MQTT 메시지를 발행하는 핸들러 빈
	 * 이 핸들러는 mqttOutboundChannel에서 수신한 메시지를 MQTT 브로커로 전송.
	 */
	@Bean
	@ServiceActivator(inputChannel = "mqttOutboundChannel")// mqttOutboundChannel에서 메시지를 처리
	public MqttPahoMessageHandler mqttOutbound() {
		MqttPahoMessageHandler messageHandler = new MqttPahoMessageHandler(clientId, mqttClientFactory());
		messageHandler.setAsync(true);// 비동기 처리 활성화
		messageHandler.setDefaultTopic(topic);// 기본 토픽 설정
		return messageHandler;
	}

	/**
	 * MQTT 메시지를 발행하기 위한 채널을 생성
	 * 이 채널은 MyGateway 인터페이스를 통해 메시지를 수신합니다.
	 */
	@Bean
	public MessageChannel mqttOutboundChannel() {
		return new DirectChannel();
	}

	/**
	 * MQTT 메시지를 발행하기 위한 게이트웨이 인터페이스
	 * 이 인터페이스를 통해 메시지를 mqttOutboundChannel로 전송합니다.
	 */
	@MessagingGateway(defaultRequestChannel = "mqttOutboundChannel")
	public interface MyGateway {
		void sendToMqtt(String data);
	}
}


