package com.be.connection.mqtt.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RestController;

import com.be.connection.mqtt.config.MqttConfig;
import com.be.connection.mqtt.service.MqttService;

/**
 * MQTT 관련 요청을 처리하는 클래스.
 */
@RestController
public class MqttController {

	private final MqttService mqttService;
	private final MqttConfig.MyGateway myGateway;

	/**
	 * MqttService와 MqttConfig의 MyGateway 인스턴스를 주입합니다.
	 * 이 인스턴스를 통해 MQTT 메시지를 발행할 수 있습니다.
	 *
	 * @param mqttService MQTT 서비스
	 * @param myGateway MQTT 메시지 게이트웨이
	 */
	@Autowired
	public MqttController(MqttService mqttService, MqttConfig.MyGateway myGateway) {
		this.mqttService = mqttService;
		this.myGateway = myGateway;
	}


	/**
	 * MQTT 메시지를 발행하는 POST 요청을 처리
	 * 이 요청은 /mqtt 엔드포인트로 전송되고 요청 본문에 포함된 메시지를 MQTT 브로커로 전송합니다.
	 * @param message 발행할 MQTT 메시지
	 * @return 리턴할 메시지 발행 성공 메시지
	 */
	@PostMapping("/mqtt")
	public String sendMqttMessage(@RequestBody String message) {
		myGateway.sendToMqtt(message);
		return "Message sent to MQTT topic.";
	}

	/**
	 * MQTT 메시지를 발행하는 GET 요청을 처리
	 * 이 요청은 /send-mqtt 엔드포인트로 전송되고, 서비스를 통해 MQTT 메시지를 발행합니다.
	 * @return 메시지 발행 성공 메시지
	 */
	@GetMapping("/send-mqtt")
	public String sendMqtt() {
		mqttService.process();
		return "MQTT message sent.";
	}
}