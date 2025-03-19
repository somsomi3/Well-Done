package com.be.connection.mqtt.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RestController;

import com.be.connection.mqtt.config.MqttConfig;

@RestController
public class MqttController {

	private final MqttConfig.MyGateway myGateway;

	@Autowired
	public MqttController(MqttConfig.MyGateway myGateway) {
		this.myGateway = myGateway;
	}

	@PostMapping("/mqtt")
	public String sendMqttMessage(@RequestBody String message) {
		myGateway.sendToMqtt(message);
		return "Message sent to MQTT topic.";
	}
}