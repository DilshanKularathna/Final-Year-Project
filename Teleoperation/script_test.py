"""
Smorphi MQTT command publisher
Sends drive commands to the ESP32 robot over HiveMQ's public broker.

Install dependency first:
    pip install paho-mqtt==1.6.1
(pinned to 1.6.1 to avoid the breaking API changes in paho-mqtt 2.x)
"""

import paho.mqtt.client as mqtt

BROKER = "broker.hivemq.com"
PORT = 1883

# Must match mqtt_topic in the ESP32 sketch EXACTLY
TOPIC = "smorphi/yourname123/control"

client = mqtt.Client()


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to {BROKER}")
    else:
        print(f"Connection failed, return code {rc}")


client.on_connect = on_connect
client.connect(BROKER, PORT, keepalive=60)
client.loop_start()

print("Smorphi MQTT Controller")
print("Commands: forward | backward | stop | quit")

try:
    while True:
        cmd = input("Enter command: ").strip().lower()
        if cmd == "quit":
            break
        elif cmd in ("forward", "backward", "stop"):
            client.publish(TOPIC, cmd)
            print(f"Sent: {cmd}")
        else:
            print("Unknown command. Use: forward, backward, stop, quit")
except KeyboardInterrupt:
    pass
finally:
    client.loop_stop()
    client.disconnect()
    print("Disconnected.")