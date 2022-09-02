#!/usr/bin/env python3


import json
import socket
import argparse
import logging

import paho.mqtt.client as mqtt


def main(udp_host, udp_port, mqtt_host):
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.bind((udp_host, udp_port))
    logging.info(f"Listening on UDP {udp_host}: {udp_port}")

    mqttc = mqtt.Client()
    mqttc.connect(mqtt_host)
    logging.info(f"Connected to MQTT Host {mqtt_host}")
    mqttc.loop_start()

    while True:
        buf = sock.recv(1024)
        data = json.loads(buf)
        topic = data['topic']
        payload = json.dumps(data['payload'])
        logging.info(f"{topic}")
        logging.debug(f"{payload}")
        mqttc.publish(topic, payload)


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
