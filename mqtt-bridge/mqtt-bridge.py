#!/usr/bin/env python3


import json
import socket
import argparse
import logging
import re

import paho.mqtt.client as mqtt


def publish_home_assistant_config(client: mqtt.Client, topic_name: str):
    logging.info(f"Publishing HA Config for '{topic_name}'")

    name = topic_name.split('/')[-1]
    title = name[:1].upper() + name[1:]
    title_split = ' '.join(re.findall('[A-Z][^A-Z]*', title))

    client.publish(
        topic=f"homeassistant/sensor/sensor{title}T/config",
        payload=json.dumps({
            "device_class": "temperature",
            "name": f"{title_split} Temperature",
            "state_topic": topic_name,
            "unit_of_measurement": "°C",
            "value_template": "{{ value_json.temperature_C }}"
        }),
        qos=1,
        retain=True)

    client.publish(
        topic=f"homeassistant/sensor/sensor{title}H/config",
        payload=json.dumps({
            "device_class": "humidity",
            "name": f"{title_split} Humidity",
            "state_topic": topic_name,
            "unit_of_measurement": "%",
            "value_template": "{{ value_json.humidity_rh }}"
        }),
        qos=1,
        retain=True)


def main(udp_host, udp_port, mqtt_host):
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.bind((udp_host, udp_port))
    logging.info(f"Listening on UDP {udp_host}: {udp_port}")

    known_topics = set()

    def on_connect(client, userdata, flags, rc):
        logging.info(f"MQTT Host {mqtt_host} Connected")
        for topic in known_topics:
            publish_home_assistant_config(client, topic)

    mqttc = mqtt.Client()
    mqttc.on_connect = on_connect
    mqttc.on_disconnect = lambda client, userdata, rc: logging.warning(
        f"MQTT Host {mqtt_host} Disconnected")
    mqttc.connect(mqtt_host)
    mqttc.loop_start()

    while True:
        buf = sock.recv(1024)
        data = json.loads(buf)
        topic = data['topic']
        payload = json.dumps(data['payload'])
        logging.debug(f"Topic \"{topic}\", Payload: {payload}")
        mqttc.publish(topic, payload)

        if not topic in known_topics:
            known_topics.add(topic)
            publish_home_assistant_config(mqttc, topic)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-l",
        "--log-level",
        default='info',
        help='Log message level - default: "info"',
        choices=['debug', 'info', 'warning', 'error', 'critical']
    )

    parser.add_argument("-u", "--udp-host",
                        default="0.0.0.0", help="UDP Bind Host")
    parser.add_argument("-p", "--udp-port", type=int,
                        default=8084, help="UDP Bind Port")
    parser.add_argument("mqtt_host", help="MQTT Broker Host")
    args = parser.parse_args()

    logging.basicConfig(
        format='%(asctime)s.%(msecs)03d | %(levelname)8s | %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S')
    logger = logging.getLogger()
    logger.setLevel(args.log_level.upper())

    main(args.udp_host, args.udp_port, args.mqtt_host)
