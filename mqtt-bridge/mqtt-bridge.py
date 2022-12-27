#!/usr/bin/env python3

import json
import socket
import argparse
import logging

import paho.mqtt.client as mqtt


def location_name_split(location: str):
    return '_'.join(location.split()).lower()


def topic_name(location: str):
    return f"data/environment-monitor/{location_name_split(location)}"


def publish_home_assistant_config(client: mqtt.Client, location: str):
    logging.info(f"Publishing HA Config for '{topic_name(location)}'")

    location_split = location_name_split(location)

    client.publish(
        topic=f"homeassistant/sensor/sensor_{location_split}_T/config",
        payload=json.dumps({
            "device_class": "temperature",
            "name": f"{location.title()} Temperature",
            "state_topic": topic_name(location),
            "unit_of_measurement": "°C",
            "value_template": "{{ value_json.temperature_C }}"
        }),
        qos=1,
        retain=True)

    client.publish(
        topic=f"homeassistant/sensor/sensor_{location_split}_H/config",
        payload=json.dumps({
            "device_class": "humidity",
            "name": f"{location.title()} Humidity",
            "state_topic": topic_name(location),
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

    known_locations = set()

    def on_connect(client, userdata, flags, rc):
        logging.info(f"MQTT Host {mqtt_host} Connected")
        for location in known_locations:
            publish_home_assistant_config(client, location)

    mqttc = mqtt.Client()
    mqttc.on_connect = on_connect
    mqttc.on_disconnect = lambda client, userdata, rc: logging.warning(
        f"MQTT Host {mqtt_host} Disconnected")
    mqttc.connect(mqtt_host)
    mqttc.loop_start()

    while True:
        buf = sock.recv(1024)
        data = json.loads(buf)
        location = data['location']
        topic = topic_name(location)

        logging.debug(f"Topic \"{topic}\", Payload: {buf}")
        mqttc.publish(topic, buf)

        if not location in known_locations:
            known_locations.add(location)
            publish_home_assistant_config(mqttc, location)


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
