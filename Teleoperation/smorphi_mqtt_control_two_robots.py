"""
Smorphi MQTT command publisher - multi-robot version
Sends drive commands to Robot 1 over HiveMQ's public broker.
Robot 1 relays r2_* commands to Robot 2 over ESP-NOW.

Install dependency first:
    pip install paho-mqtt==1.6.1
(pinned to 1.6.1 to avoid the breaking API changes in paho-mqtt 2.x)
"""

import paho.mqtt.client as mqtt

BROKER = "broker.hivemq.com"
PORT = 1883

# Must match mqtt_topic in robot1_master.ino EXACTLY
TOPIC = "smorphi/yourname123/control"

VALID_COMMANDS = {"r1_f", "r1_b", "r1_stop", "r2_f", "r2_b", "r2_stop", "quit"}

client = mqtt.Client()


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to {BROKER}")
    else:
        print(f"Connection failed, return code {rc}")


client.on_connect = on_connect
client.connect(BROKER, PORT, keepalive=60)
client.loop_start()

print("Smorphi Multi-Robot MQTT Controller")
print("Robot 1 commands: r1_f | r1_b | r1_stop")
print("Robot 2 commands: r2_f | r2_b | r2_stop")
print("quit to exit")

try:
    while True:
        cmd = input("Enter command: ").strip().lower()
        if cmd == "quit":
            break
        elif cmd in VALID_COMMANDS:
            client.publish(TOPIC, cmd)
            print(f"Sent: {cmd}")
        else:
            print("Unknown command. Use: r1_f, r1_b, r1_stop, r2_f, r2_b, r2_stop, quit")
except KeyboardInterrupt:
    pass
finally:
    client.loop_stop()
    client.disconnect()
    print("Disconnected.")
