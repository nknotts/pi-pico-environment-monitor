FROM python:3

ENV BIND_HOST 0.0.0.0
ENV BIND_PORT 8123
ENV MQTT_BROKER_HOST 127.0.0.1
WORKDIR /app

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt
COPY mqtt-bridge.py .

CMD /usr/local/bin/python3 /app/mqtt-bridge.py -u $BIND_HOST -p $BIND_PORT $MQTT_BROKER_HOST
