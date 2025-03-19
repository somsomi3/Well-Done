package com.be.connection.mqtt.service;

import org.springframework.stereotype.Service;

import com.be.connection.mqtt.config.MqttConfig;

/**
 * MQTT 메시지를 발행및 처리하는 서비스 클래스입니다.
 */
@Service
public class MqttService {

	private final MqttConfig.MyGateway myGateway;

	public MqttService(MqttConfig.MyGateway myGateway) {
		this.myGateway = myGateway;
	}

	/**
	 * MQTT 메시지를 발행하는 메서드입니다.
	 * 이 메서드는 "Hello MQTT!"라는 메시지를 MQTT 브로커로 전송합니다.
	 */
	public void process() {
		String msg = "Hello MQTT!";
		myGateway.sendToMqtt(msg);
	}
}

