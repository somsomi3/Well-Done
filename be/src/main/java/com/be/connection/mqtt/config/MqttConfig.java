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

@Configuration
public class MqttConfig {

	@Value("${spring.mqtt.broker-url}")
	private String brokerUrl;

	@Value("${spring.mqtt.client-id}")
	private String clientId;

	@Value("${spring.mqtt.topic}")
	private String topic;

	@Bean
	public MqttPahoClientFactory mqttClientFactory() {
		DefaultMqttPahoClientFactory factory = new DefaultMqttPahoClientFactory();
		MqttConnectOptions options = new MqttConnectOptions();
		options.setServerURIs(new String[]{brokerUrl});
		factory.setConnectionOptions(options);
		return factory;
	}

	@Bean
	@ServiceActivator(inputChannel = "mqttOutboundChannel")
	public MqttPahoMessageHandler mqttOutbound() {
		MqttPahoMessageHandler messageHandler = new MqttPahoMessageHandler(clientId, mqttClientFactory());
		messageHandler.setAsync(true);
		messageHandler.setDefaultTopic(topic);
		return messageHandler;
	}

	@Bean
	public MessageChannel mqttOutboundChannel() {
		return new DirectChannel();
	}

	@MessagingGateway(defaultRequestChannel = "mqttOutboundChannel")
	public interface MyGateway {
		void sendToMqtt(String data);
	}
}


