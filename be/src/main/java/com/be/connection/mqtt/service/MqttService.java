package com.be.connection.mqtt.service;

import org.springframework.stereotype.Service;

import com.be.connection.mqtt.config.MqttConfig;

@Service
public class MqttService {

	private final MqttConfig.MyGateway myGateway;

	public MqttService(MqttConfig.MyGateway myGateway) {
		this.myGateway = myGateway;
	}

	public void process() {
		String msg = "Hello MQTT!";
		myGateway.sendToMqtt(msg);
	}
}

